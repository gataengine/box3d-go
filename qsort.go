// Port of src/qsort.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// Copyright (c) 2013, 2017 Alexey Tourbin
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package box3d

/*
 * This is a traditional Quicksort implementation which mostly follows
 * [Sedgewick 1978].  Sorting is performed entirely on array indices,
 * while actual access to the array elements is abstracted out with the
 * user-defined `LESS` and `SWAP` primitives.
 *
 * Synopsis:
 *	QSORT(N, LESS, SWAP);
 * where
 *	N - the number of elements in A[];
 *	LESS(i, j) - compares A[i] to A[j];
 *	SWAP(i, j) - exchanges A[i] with A[j].
 */

// PORT: the macros become functions taking the LESS and SWAP primitives as closures. Indices are
// int instead of unsigned long; no index ever wraps (the partition leaves q_j >= q_l + 1 before its
// final decrement), so the comparisons and the sequence of LESS/SWAP calls are unchanged. The
// sort is not stable, so the exact sequence matters wherever keys repeat (mesh_contact.c).

// qSort3 is Q_SORT3: sort 3 elements.
func qSort3(a1, a2, a3 int, less func(i, j int) bool, swap func(i, j int)) {
	if less(a2, a1) {
		if less(a3, a2) {
			swap(a1, a3)
		} else {
			swap(a1, a2)
			if less(a3, a2) {
				swap(a2, a3)
			}
		}
	} else if less(a3, a2) {
		swap(a2, a3)
		if less(a2, a1) {
			swap(a1, a2)
		}
	}
}

// qPartition is Q_PARTITION: partition [q_l,q_r] around a pivot. After partitioning,
// [q_l,q_j] are the elements that are less than or equal to the pivot,
// while [q_i,q_r] are the elements greater than or equal to the pivot.
func qPartition(l, r int, less func(i, j int) bool, swap func(i, j int)) (i, j int) {
	// The middle element, not to be confused with the median.
	m := l + ((r - l) >> 1)
	// Reorder the second, the middle, and the last items.
	// As [Edelkamp Weiss 2016] explain, using the second element
	// instead of the first one helps avoid bad behaviour for
	// decreasingly sorted arrays.  This method is used in recent
	// versions of gcc's std::sort, see gcc bug 58437#c13, although
	// the details are somewhat different (cf. #c14).
	qSort3(l+1, m, r, less, swap)
	// Place the median at the beginning.
	swap(l, m)
	// Partition [q_l+2, q_r-1] around the median which is in q_l.
	// q_i and q_j are initially off by one, they get decremented
	// in the do-while loops.
	i = l + 1
	j = r
	for {
		for {
			i++
			if !less(i, l) {
				break
			}
		}
		for {
			j--
			if !less(l, j) {
				break
			}
		}
		if i >= j {
			break // Sedgewick says "until j < i"
		}
		swap(i, j)
	}
	// Compensate for the i==j case.
	i = j + 1
	// Put the median to its final place.
	swap(l, j)
	// The median is not part of the left subfile.
	j--
	return i, j
}

// qInsertionSort is Q_INSERTION_SORT.
// Insertion sort is applied to small subfiles - this is contrary to
// Sedgewick's suggestion to run a separate insertion sort pass after
// the partitioning is done.  The reason I don't like a separate pass
// is that it triggers extra comparisons, because it can't see that the
// medians are already in their final positions and need not be rechecked.
// Since I do not assume that comparisons are cheap, I also do not try
// to eliminate the (q_j > q_l) boundary check.
func qInsertionSort(l, r int, less func(i, j int) bool, swap func(i, j int)) {
	// For each item starting with the second...
	for i := l + 1; i <= r; i++ {
		// move it down the array so that the first part is sorted.
		for j := i; j > l && less(j, j-1); j-- {
			swap(j, j-1)
		}
	}
}

// qThresh is Q_THRESH.
// When the size of [q_l,q_r], i.e. q_r-q_l+1, is greater than or equal to
// Q_THRESH, the algorithm performs recursive partitioning.  When the size
// drops below Q_THRESH, the algorithm switches to insertion sort.
// The minimum valid value is probably 5 (with 5 items, the second and
// the middle items, the middle itself being rounded down, are distinct).
const qThresh = 16

// qStackSize is the Q_LOOP stack size for a 32-bit Q_N, which is what every upstream call site
// passes (an int count).
// On 32-bit platforms, to sort a "char[3GB+]" array,
// it may take full 32 stack frames.  On 64-bit CPUs,
// though, the address space is limited to 48 bits.
// The usage is further reduced if Q_N has a 32-bit type.
const qStackSize = 32

// qLoop is Q_LOOP: the main loop, with Q_SUBFILES inlined at both call sites.
func qLoop(n int, less func(i, j int) bool, swap func(i, j int)) {
	l := 0
	r := n - 1
	sp := 0 // the number of frames pushed to the stack
	var st [qStackSize]struct{ l, r int }

	// subfiles is Q_SUBFILES, the missing part: dealing with subfiles.
	// Assumes that the first subfile is not smaller than the second.
	subfiles := func(l1, r1, l2, r2 int) {
		// If the second subfile is only a single element, it needs
		// no further processing.  The first subfile will be processed
		// on the next iteration (both subfiles cannot be only a single
		// element, due to Q_THRESH).
		if l2 == r2 {
			l = l1
			r = r1
		} else {
			// Otherwise, both subfiles need processing.
			// Push the larger subfile onto the stack.
			st[sp].l = l1
			st[sp].r = r1
			sp++
			// Process the smaller subfile on the next iteration.
			l = l2
			r = r2
		}
	}

	for {
		if r-l+1 >= qThresh {
			i, j := qPartition(l, r, less, swap)
			// Now have two subfiles: [q_l,q_j] and [q_i,q_r].
			// Dealing with them depends on which one is bigger.
			if j-l >= r-i {
				subfiles(l, j, i, r)
			} else {
				subfiles(i, r, l, j)
			}
		} else {
			qInsertionSort(l, r, less, swap)
			// Pop subfiles from the stack, until it gets empty.
			if sp == 0 {
				break
			}
			sp--
			l = st[sp].l
			r = st[sp].r
		}
	}
}

// qsort is QSORT( N, LESS, SWAP ).
// And now, ladies and gentlemen, may I proudly present to you...
func qsort(n int, less func(i, j int) bool, swap func(i, j int)) {
	if n > 1 {
		// We could check sizeof(Q_N) and use "unsigned", but at least
		// on x86_64, this has the performance penalty of up to 5%.
		qLoop(n, less, swap)
	}
}
