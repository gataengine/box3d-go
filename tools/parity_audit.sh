#!/usr/bin/env bash
# Parity audit: every public Box3D symbol and the Go name it maps to.
#
# Usage (from the repo root or libs/box3d-go):
#   libs/box3d-go/tools/parity_audit.sh [-v] [path-to-box3d-checkout]
#
# The upstream checkout defaults to third_party/box3d of the enclosing repo. Lists every B3_API
# function, every B3_INLINE function, every public type and enum value of include/box3d/*.h, maps
# each to its Go name by the port's naming rules (README.md, "API mapping") and checks that the
# Go package declares it. Symbols intentionally not ported are listed in KNOWN_OMISSIONS below;
# they are reported but do not fail the audit. -v prints every mapping.
#
# Exit status: 0 when nothing is missing, 2 when a symbol is missing, 1 on usage errors.
# Portable: bash 3.2 (macOS) and Linux; needs perl, grep, sort, awk.
set -euo pipefail

VERBOSE=0
if [ "${1:-}" = "-v" ]; then
  VERBOSE=1
  shift
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "$ROOT/../.." && pwd)"
UPSTREAM="${1:-$REPO/third_party/box3d}"
if [ $# -eq 0 ] && [ ! -d "$UPSTREAM/include/box3d" ]; then
  # A linked git worktree may not have the submodule checked out; try the main worktree's.
  common="$(git -C "$REPO" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)"
  if [ -n "$common" ] && [ -d "$(dirname "$common")/third_party/box3d/include/box3d" ]; then
    UPSTREAM="$(dirname "$common")/third_party/box3d"
  fi
fi
HDR="$UPSTREAM/include/box3d"

if [ ! -d "$HDR" ]; then
  echo "upstream include dir not found: $HDR" >&2
  echo "pass the Box3D checkout: $0 [-v] path/to/box3d (git submodule update --init third_party/box3d)" >&2
  exit 1
fi

# Public C symbols that have no Go counterpart on purpose. One per line: symbol, then the reason.
# Keep in sync with PARITY_REPORT.md.
KNOWN_OMISSIONS='
b3SetAllocator allocator hooks: Go memory is garbage collected
b3AllocFcn allocator hook type: Go memory is garbage collected
b3FreeFcn allocator hook type: Go memory is garbage collected
b3InternalAssert assert plumbing: B3_ASSERT is a Go function (base_assert.go)
b3World_DumpShapeBounds declared in box3d.h but never defined upstream
B3_ENABLE_VALIDATION compile switch: the box3d_validate build tag (validateEnabled)
'

tmp="$(mktemp -d 2>/dev/null || mktemp -d -t box3d-parity)"
trap 'rm -rf "$tmp"' EXIT

# --- C side -------------------------------------------------------------------------------------

# B3_API functions (declarations may wrap lines).
perl -0777 -ne 'while(/^\s*B3_API\s+[^;{]*?\b(b3[A-Za-z0-9_]+)\s*\(/mg){print "$1\n"}' \
  "$HDR"/*.h | sort -u > "$tmp/c_api.txt"

# B3_INLINE functions of the public headers (math_functions.h, collision.h, ...).
perl -0777 -ne 'while(/^\s*B3_INLINE\s+[^;{(]*?\b(b3[A-Za-z0-9_]+)\s*\(/mg){print "$1\n"}' \
  "$HDR"/*.h | sort -u > "$tmp/c_inline.txt"

# Public types: typedef struct/enum/union names, plain typedefs and function (pointer) typedefs.
perl -0777 -ne '
  while(/typedef\s+(?:struct|enum|union)\s+(b3[A-Za-z0-9_]+)/g){print "$1\n"}
  while(/typedef\s+[^;()]*\(\s*(b3[A-Za-z0-9_]+)\s*\)\s*\(/g){print "$1\n"}
  while(/typedef\s+[A-Za-z0-9_ ]+\s+(b3[A-Za-z0-9_]+)\s*;/g){print "$1\n"}
  while(/typedef\s+[A-Za-z0-9_ *]+?\b(b3[A-Za-z0-9_]+)\s*\(/g){print "$1\n"}
' "$HDR"/*.h | sort -u > "$tmp/c_types.txt"

# Enum values: b3_xxx identifiers declared inside typedef enum bodies.
perl -0777 -ne '
  while(/typedef\s+enum\s+\w+\s*\{(.*?)\}/sg){
    my $b=$1; $b=~s{//[^\n]*}{}g; $b=~s{/\*.*?\*/}{}sg;
    while($b=~/\b(b3_[A-Za-z0-9_]+)\b\s*(?:=|,|$)/mg){print "$1\n"}
  }' "$HDR"/*.h | sort -u > "$tmp/c_enums.txt"

# Object-like B3_ constants with a value (numbers, parenthesised expressions, UINT64_MAX or
# another B3_ constant). Compiler/attribute macros (B3_API, B3_INLINE, ...) have no such value.
perl -ne '
  if(/^\s*#\s*define\s+(B3_[A-Z0-9_]+)\s+(\(|-|[0-9]|UINT64_MAX|B3_[A-Z0-9_]+\s*$)/){print "$1\n"}
' "$HDR"/*.h | sort -u > "$tmp/c_consts.txt"

# --- Go side ------------------------------------------------------------------------------------

gofiles="$tmp/gofiles.txt"
ls "$ROOT"/*.go | grep -v '_test\.go$' > "$gofiles"

# Exported funcs as "Name" and exported methods as "Recv.Name" (pointer and type params stripped).
xargs cat < "$gofiles" | perl -ne '
  if(/^func\s+\(\s*\w*\s*\*?\s*([A-Za-z0-9_]+)(?:\[[^\]]*\])?\s*\)\s*([A-Z]\w*)/){print "$1.$2\n"}
  elsif(/^func\s+([A-Z]\w*)/){print "$1\n"}
' | sort -u > "$tmp/go_funcs.txt"

# Exported type names, consts and vars (block and single-line forms).
xargs cat < "$gofiles" | perl -ne '
  if(/^type\s+([A-Z]\w*)/){print "$1\n"; next}
  if(/^(const|var|type)\s*\(\s*$/){$blk=$1; next}
  if($blk && /^\)/){$blk=""; next}
  if($blk eq "type" && /^\t([A-Z]\w*)\s/){print "$1\n"; next}
  if($blk && /^\t([A-Z]\w*)\b/){print "$1\n"; next}
  if(/^(?:const|var)\s+([A-Z]\w*)/){print "$1\n"}
' | sort -u > "$tmp/go_names.txt"

has_func() { grep -qxF "$1" "$tmp/go_funcs.txt"; }
has_name() { grep -qxF "$1" "$tmp/go_names.txt"; }

omission_reason() {
  printf '%s\n' "$KNOWN_OMISSIONS" | awk -v s="$1" '$1==s {$1=""; sub(/^ /,""); print; exit}'
}

# Id types whose b3Type_Method functions become methods on <Type>ID.
id_type() {
  case "$1" in
    World|Body|Shape|Joint|Contact) echo "${1}ID" ;;
    *) echo "" ;;
  esac
}

# Candidate Go names for a C function, best first. The first one found wins.
candidates() {
  local sym="$1" core="${1#b3}" type method idt
  case "$core" in
    *_*)
      type="${core%%_*}"
      method="${core#*_}"
      idt="$(id_type "$type")"
      if [ -n "$idt" ]; then
        echo "$idt.$method"                       # b3Body_SetTransform -> BodyID.SetTransform
      fi
      case "$type" in
        *Joint) echo "JointID.$type$method" ;;    # b3RevoluteJoint_SetSpringHertz -> JointID.RevoluteJointSetSpringHertz
      esac
      echo "$type.$method"                        # b3RecPlayer_StepFrame -> (*RecPlayer).StepFrame
      echo "$method$type"                         # b3DynamicTree_Create -> CreateDynamicTree (constructors)
      echo "$type$method"                         # free function fallback
      ;;
    Create*Joint) echo "WorldID.$core" ;;         # b3CreateRevoluteJoint -> WorldID.CreateRevoluteJoint
    CreateBody)   echo "WorldID.CreateBody" ;;
    Create*Shape) echo "BodyID.$core" ;;          # b3CreateHullShape -> BodyID.CreateHullShape
    DestroyWorld) echo "WorldID.Destroy" ;;
    DestroyBody)  echo "BodyID.Destroy" ;;
    DestroyShape) echo "ShapeID.Destroy" ;;
    DestroyJoint) echo "JointID.Destroy" ;;
    DestroyPlayer) echo "RecPlayer.Destroy" ;;
    Destroy*)     echo "${core#Destroy}.Destroy" ;;
  esac
  echo "$core"                                    # b3MakeBoxHull -> MakeBoxHull
}

# Candidate Go names for a C type, enum value or B3_ constant, best first. Go initialisms are
# upper case: b3BodyId -> BodyID, b3_toiStateHit -> TOIStateHit, B3_MAX_MANIFOLD_POINTS ->
# MaxManifoldPoints.
type_names() {
  local core
  case "$1" in
    B3_*)
      core="$(printf '%s\n' "${1#B3_}" | perl -pe '$_=join("", map { ucfirst(lc($_)) } split(/_/, $_)); $_.="\n"')"
      ;;
    *)
      core="${1#b3}"
      core="${core#_}"
      core="$(printf '%s\n' "$core" | perl -pe 's/^(.)/\u$1/')"
      ;;
  esac
  echo "$core"
  printf '%s\n' "$core" | perl -pe 's/Id(?=[A-Z0-9]|$)/ID/g; s/^Toi(?=[A-Z])/TOI/; s/Aabb/AABB/g; s/Sse2/SSE2/g'
}

missing_total=0
omitted_total=0
mapped_total=0

audit_funcs() {
  local label="$1" list="$2" sym found c n=0 miss=0 omit=0 why
  : > "$tmp/miss_$label.txt"
  while read -r sym; do
    [ -n "$sym" ] || continue
    n=$((n+1))
    found=""
    for c in $(candidates "$sym"); do
      if has_func "$c"; then found="$c"; break; fi
    done
    if [ -z "$found" ]; then
      why="$(omission_reason "$sym")"
      if [ -n "$why" ]; then
        omit=$((omit+1))
        [ "$VERBOSE" = 1 ] && printf '  %-48s omitted (%s)\n' "$sym" "$why"
      else
        miss=$((miss+1))
        echo "$sym -> $(candidates "$sym" | head -1)" >> "$tmp/miss_$label.txt"
      fi
    else
      [ "$VERBOSE" = 1 ] && printf '  %-48s %s\n' "$sym" "$found"
    fi
  done < "$list"
  report "$label" "$n" "$miss" "$omit"
}

audit_names() {
  local label="$1" list="$2" sym g found n=0 miss=0 omit=0 why
  : > "$tmp/miss_$label.txt"
  while read -r sym; do
    [ -n "$sym" ] || continue
    n=$((n+1))
    found=""
    for g in $(type_names "$sym"); do
      # Constants scaled by the length unit (B3_LINEAR_SLOP, ...) are functions: LinearSlop().
      if has_name "$g" || has_func "$g"; then found="$g"; break; fi
    done
    if [ -n "$found" ]; then
      [ "$VERBOSE" = 1 ] && printf '  %-48s %s\n' "$sym" "$found"
      continue
    fi
    g="$(type_names "$sym" | tail -1)"
    why="$(omission_reason "$sym")"
    if [ -n "$why" ]; then
      omit=$((omit+1))
      [ "$VERBOSE" = 1 ] && printf '  %-48s omitted (%s)\n' "$sym" "$why"
    else
      miss=$((miss+1))
      echo "$sym -> $g" >> "$tmp/miss_$label.txt"
    fi
  done < "$list"
  report "$label" "$n" "$miss" "$omit"
}

report() {
  local label="$1" n="$2" miss="$3" omit="$4"
  echo "$label: total=$n mapped=$((n-miss-omit)) omitted=$omit missing=$miss"
  if [ "$miss" -ne 0 ]; then
    sed 's/^/  MISSING /' "$tmp/miss_$label.txt"
  fi
  missing_total=$((missing_total+miss))
  omitted_total=$((omitted_total+omit))
  mapped_total=$((mapped_total+n-miss-omit))
}

echo "upstream: $UPSTREAM ($(git -C "$UPSTREAM" rev-parse --short=8 HEAD 2>/dev/null || echo unknown))"
echo "go package: $ROOT"
[ "$VERBOSE" = 1 ] && echo "B3_API functions:"
audit_funcs b3_api "$tmp/c_api.txt"
[ "$VERBOSE" = 1 ] && echo "B3_INLINE functions:"
audit_funcs b3_inline "$tmp/c_inline.txt"
[ "$VERBOSE" = 1 ] && echo "types:"
audit_names types "$tmp/c_types.txt"
[ "$VERBOSE" = 1 ] && echo "enum values:"
audit_names enums "$tmp/c_enums.txt"
[ "$VERBOSE" = 1 ] && echo "constants:"
audit_names constants "$tmp/c_consts.txt"
echo "---"
echo "go_exported_funcs_and_methods=$(wc -l < "$tmp/go_funcs.txt" | tr -d ' ')"
echo "mapped=$mapped_total omitted=$omitted_total missing=$missing_total"

if [ "$missing_total" -ne 0 ]; then
  exit 2
fi
