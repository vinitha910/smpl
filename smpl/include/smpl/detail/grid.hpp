#ifndef SMPL_GRID_HPP
#define SMPL_GRID_HPP

#include "../grid.h"

namespace sbpl {

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid()
{
}

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid(const Allocator& alloc)
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid(
    size_type xdim,
    size_type ydim,
    size_type zdim,
    const T& value,
    const Allocator& alloc)
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid(
    size_type xdim,
    size_type ydim,
    size_type zdim,
    const Allocator& alloc)
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid(const Grid& other)
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid(const Grid& other, const Allocator& alloc)
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid(Grid&& other)
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>::Grid(Grid&& other, const Allocator& alloc)
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>::~Grid()
{

}

template <typename T, typename Allocator>
Grid<T, Allocator>& Grid<T, Allocator>::operator=(const Grid& other)
{
    return *this;
}

template <typename T, typename Allocator>
Grid<T, Allocator>& Grid<T, Allocator>::operator=(Grid&& other)
{
    return *this;
}

template <typename T, typename Allocator>
void Grid<T, Allocator>::assign(
    size_type xdim,
    size_type ydim,
    size_type zdim,
    const T& value)
{

}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::allocator_type
Grid<T, Allocator>::get_allocator() const
{
    return m_alloc;
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::reference
Grid<T, Allocator>::at(size_type x, size_type y, size_type z)
{
    return *m_data;
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_reference
Grid<T, Allocator>::at(size_type x, size_type y, size_type z) const
{
    return *m_data;
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::reference
Grid<T, Allocator>::operator[](size_type pos)
{
    return *m_data;
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_reference
Grid<T, Allocator>::operator[](size_type pos) const
{
    return *m_data;
}

template <typename T, typename Allocator>
T* Grid<T, Allocator>::data()
{
    return m_data;
}

template <typename T, typename Allocator>
const T* Grid<T, Allocator>::data() const
{
    return m_data;
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::iterator Grid<T, Allocator>::begin()
{
    return iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_iterator Grid<T, Allocator>::begin() const
{
    return const_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_iterator Grid<T, Allocator>::cbegin() const
{
    return const_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::iterator Grid<T, Allocator>::end()
{
    return iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_iterator Grid<T, Allocator>::end() const
{
    return const_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_iterator Grid<T, Allocator>::cend() const
{
    return const_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::reverse_iterator Grid<T, Allocator>::rbegin()
{
    return reverse_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_reverse_iterator
Grid<T, Allocator>::rbegin() const
{
    return const_reverse_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_reverse_iterator
Grid<T, Allocator>::crbegin() const
{
    return const_reverse_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::reverse_iterator Grid<T, Allocator>::rend()
{
    return reverse_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_reverse_iterator
Grid<T, Allocator>::rend() const
{
    return const_reverse_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::const_reverse_iterator
Grid<T, Allocator>::crend() const
{
    return const_reverse_iterator();
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::size_type Grid<T, Allocator>::size() const
{
    return 0;
}

template <typename T, typename Allocator>
typename Grid<T, Allocator>::size_type Grid<T, Allocator>::max_size() const
{
    return 0;
}

template <typename T, typename Allocator>
void Grid<T, Allocator>::clear()
{

}

template <typename T, typename Allocator>
void Grid<T, Allocator>::resize(size_type count)
{

}

template <typename T, typename Allocator>
void Grid<T, Allocator>::resize(size_type count, const value_type& value)
{

}

template <typename T, typename Allocator>
void Grid<T, Allocator>::swap(Grid& other)
{

}

} // namespace sbpl

#endif
