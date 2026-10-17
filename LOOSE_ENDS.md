# Loose ends

Every place the Go port is not a literal transliteration of upstream Box3D
(`efca0c184d40fd6a82016983f1601c4d23965867`). Each entry has a matching `// PORT:` comment at the
site. Upstream bugs are ported as is and listed here too.

| Go file | upstream site | what differs | why |
|---|---|---|---|
| test_macros_test.go | `RUN_TEST`, `RUN_SUBTEST` | replaced by Ginkgo `Describe`/`It` | Go test runner |
| box3d_suite_test.go | `test/main.c` | filter argument is `-ginkgo.focus`; Windows CRT and Tracy setup dropped | Go test runner |
