#ifndef SMPL_GRID_H
#define SMPL_GRID_H

#include <memory>

namespace sbpl {

template <typename T, typename Allocator = std::allocator<T>>
class Grid
{
public:

    typedef T           value_type;
    typedef Allocator   allocator_type;

    typedef size_t          size_type;
    typedef std::ptrdiff_t  difference_type;

    typedef value_type&         reference;
    typedef const value_type&   const_reference;

    typedef std::allocator_traits<Allocator>::pointer       pointer;
    typedef std::allocator_traits<Allocator>::const_pointer const_pointer;

    typedef pointer         iterator;
    typedef const_pointer   const_iterator;
    typedef pointer         reverse_iterator;
    typedef const_pointer   const_reverse_iterator;

    Grid();

    explicit Grid(const Allocator& alloc);

    Grid(
        size_type xdim,
        size_type ydim,
        size_type zdim,
        const T& value,
        const Allocator& alloc = Allocator());

    explicit Grid(
        size_type xdim,
        size_type ydim,
        size_type zdim,
        const Allocator& alloc = Allocator());

    Grid(const Grid& other);
    Grid(const Grid& other, const Allocator& alloc);

    Grid(Grid&& other);
    Grid(Grid&& other, const Allocator& alloc);

    ~Grid();

    Grid& operator=(const Grid& other);
    Grid& operator=(Grid&& other);

    void assign(size_type xdim, size_type ydim, size_type zdim, const T& value);

    allocator_type get_allocator() const;

    reference       at(size_type x, size_type y, size_type z);
    const_reference at(size_type x, size_type y, size_type z) const;

    reference       operator[](size_type pos);
    const_reference operator[](size_type pos) const;

    T* data();
    const T* data() const;

    iterator begin();
    const_iterator begin() const;
    const_iterator cbegin() const;

    iterator end();
    const_iterator end() const;
    const_iterator cend() const;

    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    const_reverse_iterator crbegin() const;

    reverse_iterator rend();
    const_reverse_iterator rend() const;
    const_reverse_iterator crend() const;

    size_type size() const;
    size_type max_size() const;

    void clear();

    void resize(size_type count);
    void resize(size_type count, const value_type& value);

    void swap(Grid& other);

private:

    value_type* m_data;
    allocator_type m_alloc;
};

template <typename T, typename Alloc>
void swap(Grid<T, Alloc>& lhs, Grid<T, Alloc>& rhs);

} // namespace sbpl

#include "detail/grid.hpp"

#endif
