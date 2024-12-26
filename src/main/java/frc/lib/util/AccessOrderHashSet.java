/*
 * Copied from https://github.com/frohoff/jdk8u-jdk/blob/master/src/share/classes/java/util/HashSet.java
 * Copyright (c) 1997, 2013, Oracle and/or its affiliates. All rights reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.  Oracle designates this
 * particular file as subject to the "Classpath" exception as provided
 * by Oracle in the LICENSE file that accompanied this code.
 *
 * This code is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * version 2 for more details (a copy is included in the LICENSE file that
 * accompanied this code).
 *
 * You should have received a copy of the GNU General Public License version
 * 2 along with this work; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Please contact Oracle, 500 Oracle Parkway, Redwood Shores, CA 94065 USA
 * or visit www.oracle.com if you need additional information or have any
 * questions.
 */

package frc.lib.util;

import java.util.AbstractSet;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Set;

/**
 * This class is the same as the LinkedHashSet except for the behavior when accessed or an element
 * is re-inserted in the list. This this case the element <i>is</> reordered to the end of the list.
 *
 * <p>Only other difference is this does not implement Serializeable, and does not offer the
 * spliterator method.
 *
 * @param <E> the type of elements maintained by this set
 * @author Josh Bloch
 * @author Neal Gafter
 * @see Collection
 * @see Set
 * @see TreeSet
 * @see HashMap
 * @since 1.2
 */
public class AccessOrderHashSet<E> extends AbstractSet<E> implements Set<E>, Cloneable {
  private transient LinkedHashMap<E, Object> map;

  // Dummy value to associate with an Object in the backing Map
  private static final Object PRESENT = new Object();

  /**
   * Constructs a new, empty set; the backing <tt>HashMap</tt> instance has default initial capacity
   * (16) and load factor (0.75).
   */
  public AccessOrderHashSet() {
    map = new LinkedHashMap<>(16, 0.75f, true);
  }

  /**
   * Constructs a new set containing the elements in the specified collection. The <tt>HashMap</tt>
   * is created with default load factor (0.75) and an initial capacity sufficient to contain the
   * elements in the specified collection.
   *
   * @param c the collection whose elements are to be placed into this set
   * @throws NullPointerException if the specified collection is null
   */
  public AccessOrderHashSet(Collection<? extends E> c) {
    map = new LinkedHashMap<>(Math.max((int) (c.size() / .75f) + 1, 16), 0.75f, true);
    addAll(c);
  }

  /**
   * Constructs a new, empty set; the backing <tt>HashMap</tt> instance has the specified initial
   * capacity and the specified load factor.
   *
   * @param initialCapacity the initial capacity of the hash map
   * @param loadFactor the load factor of the hash map
   * @throws IllegalArgumentException if the initial capacity is less than zero, or if the load
   *     factor is nonpositive
   */
  public AccessOrderHashSet(int initialCapacity, float loadFactor) {
    map = new LinkedHashMap<>(initialCapacity, loadFactor, true);
  }

  /**
   * Constructs a new, empty set; the backing <tt>HashMap</tt> instance has the specified initial
   * capacity and default load factor (0.75).
   *
   * @param initialCapacity the initial capacity of the hash table
   * @throws IllegalArgumentException if the initial capacity is less than zero
   */
  public AccessOrderHashSet(int initialCapacity) {
    map = new LinkedHashMap<>(initialCapacity, 0.75f, true);
  }

  /**
   * Constructs a new, empty linked hash set. (This package private constructor is only used by
   * LinkedHashSet.) The backing HashMap instance is a LinkedHashMap with the specified initial
   * capacity and the specified load factor.
   *
   * @param initialCapacity the initial capacity of the hash map
   * @param loadFactor the load factor of the hash map
   * @param dummy ignored (distinguishes this constructor from other int, float constructor.)
   * @throws IllegalArgumentException if the initial capacity is less than zero, or if the load
   *     factor is nonpositive
   */
  AccessOrderHashSet(int initialCapacity, float loadFactor, boolean dummy) {
    map = new LinkedHashMap<>(initialCapacity, loadFactor, true);
  }

  /**
   * Returns an iterator over the elements in this set. The elements are returned in no particular
   * order.
   *
   * @return an Iterator over the elements in this set
   * @see ConcurrentModificationException
   */
  public Iterator<E> iterator() {
    return map.keySet().iterator();
  }

  /**
   * Returns the number of elements in this set (its cardinality).
   *
   * @return the number of elements in this set (its cardinality)
   */
  public int size() {
    return map.size();
  }

  /**
   * Returns <tt>true</tt> if this set contains no elements.
   *
   * @return <tt>true</tt> if this set contains no elements
   */
  public boolean isEmpty() {
    return map.isEmpty();
  }

  /**
   * Returns <tt>true</tt> if this set contains the specified element. More formally, returns
   * <tt>true</tt> if and only if this set contains an element <tt>e</tt> such that
   * <tt>(o==null&nbsp;?&nbsp;e==null&nbsp;:&nbsp;o.equals(e))</tt>.
   *
   * @param o element whose presence in this set is to be tested
   * @return <tt>true</tt> if this set contains the specified element
   */
  public boolean contains(Object o) {
    return map.containsKey(o);
  }

  /**
   * Adds the specified element to this set if it is not already present. More formally, adds the
   * specified element <tt>e</tt> to this set if this set contains no element <tt>e2</tt> such that
   * <tt>(e==null&nbsp;?&nbsp;e2==null&nbsp;:&nbsp;e.equals(e2))</tt>. If this set already contains
   * the element, the call leaves the set unchanged and returns <tt>false</tt>.
   *
   * @param e element to be added to this set
   * @return <tt>true</tt> if this set did not already contain the specified element
   */
  public boolean add(E e) {
    return map.put(e, PRESENT) == null;
  }

  /**
   * Removes the specified element from this set if it is present. More formally, removes an element
   * <tt>e</tt> such that <tt>(o==null&nbsp;?&nbsp;e==null&nbsp;:&nbsp;o.equals(e))</tt>, if this
   * set contains such an element. Returns <tt>true</tt> if this set contained the element (or
   * equivalently, if this set changed as a result of the call). (This set will not contain the
   * element once the call returns.)
   *
   * @param o object to be removed from this set, if present
   * @return <tt>true</tt> if the set contained the specified element
   */
  public boolean remove(Object o) {
    return map.remove(o) == PRESENT;
  }

  /** Removes all of the elements from this set. The set will be empty after this call returns. */
  public void clear() {
    map.clear();
  }

  /**
   * Returns a shallow copy of this <tt>HashSet</tt> instance: the elements themselves are not
   * cloned.
   *
   * @return a shallow copy of this set
   */
  @SuppressWarnings("unchecked")
  public Object clone() {
    try {
      AccessOrderHashSet<E> newSet = (AccessOrderHashSet<E>) super.clone();
      newSet.map = (LinkedHashMap<E, Object>) map.clone();
      return newSet;
    } catch (CloneNotSupportedException e) {
      throw new InternalError(e);
    }
  }

  /**
   * Creates a <em><a href="Spliterator.html#binding">late-binding</a></em> and <em>fail-fast</em>
   * {@link Spliterator} over the elements in this set.
   *
   * <p>The {@code Spliterator} reports {@link Spliterator#SIZED} and {@link Spliterator#DISTINCT}.
   * Overriding implementations should document the reporting of additional characteristic values.
   *
   * @return a {@code Spliterator} over the elements in this set
   * @since 1.8
   */
  // public Spliterator<E> spliterator() {
  //    return new LinkedHashMap.KeySpliterator<E,Object>(map, 0, -1, 0, 0);
  // }
}
