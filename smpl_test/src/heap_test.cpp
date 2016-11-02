#include <stdio.h>
#include <iostream>
#include <type_traits>
#include <vector>

#define BOOST_TEST_MODULE HeapTest
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/container/stable_vector.hpp>

#include <smpl/intrusive_heap.h>

struct open_element : sbpl::heap_element
{
    int priority;

    open_element() = default;
    open_element(int p) : priority(p) { }
};

struct open_element_compare
{
    bool operator()(const sbpl::heap_element* a, const sbpl::heap_element* b)
    {
        return static_cast<const open_element*>(a)->priority <
                static_cast<const open_element*>(b)->priority;
    }
};

typedef sbpl::intrusive_heap<open_element_compare> heap_type;

BOOST_AUTO_TEST_CASE(DefaultConstructTest)
{
    heap_type h;
    BOOST_CHECK(h.empty());
    BOOST_CHECK(h.size() == 0);
    BOOST_CHECK(h.begin() == h.end());

    open_element_compare comp;
    heap_type h2(comp);
    BOOST_CHECK(h2.empty());
    BOOST_CHECK(h2.size() == 0);
    BOOST_CHECK(h2.begin() == h2.end());
}

BOOST_AUTO_TEST_CASE(ElementConstructorTest)
{
    open_element no_elements[] = { };
    open_element one_elements[] = { 2 };
    open_element few_elements[] = { 8, 10, 4, 2, 12 };

    open_element* no_element_ptrs[] = { };
    open_element* one_element_ptrs[] = { &one_elements[0] };
    open_element* few_elements_ptrs[] = {
        &few_elements[0],
        &few_elements[1],
        &few_elements[2],
        &few_elements[3],
        &few_elements[4],
    };

    open_element** no_element_ptrs_begin = no_element_ptrs;
    open_element** no_element_ptrs_end =
            no_element_ptrs + sizeof(no_element_ptrs) / sizeof(open_element*);
    heap_type h(no_element_ptrs_begin, no_element_ptrs_end);
    BOOST_CHECK(h.empty());
    BOOST_CHECK(h.size() == 0);
    BOOST_CHECK(h.begin() == h.end());

    open_element** one_element_ptrs_begin = one_element_ptrs;
    open_element** one_element_ptrs_end =
            one_element_ptrs + sizeof(one_element_ptrs) / sizeof(open_element*);
    heap_type h1(one_element_ptrs_begin, one_element_ptrs_end);
    BOOST_CHECK(!h1.empty());
    BOOST_CHECK(h1.size() == 1);
    BOOST_CHECK(std::distance(h1.begin(), h1.end()) == 1);
    BOOST_CHECK(h1.min() == &one_elements[0]);

    open_element** few_element_ptrs_begin = few_elements_ptrs;
    open_element** few_element_ptrs_end =
            few_elements_ptrs + sizeof(few_elements_ptrs) / sizeof(open_element*);
    heap_type h2(few_element_ptrs_begin, few_element_ptrs_end);
    BOOST_CHECK(!h2.empty());
    BOOST_CHECK(h2.size() == std::distance(few_element_ptrs_begin, few_element_ptrs_end));
    BOOST_CHECK(h2.min() == &few_elements[3]);
}

template <typename Iterator>
class pointer_iterator :
    public std::iterator<
            typename std::iterator_traits<Iterator>::iterator_category,
            typename std::iterator_traits<Iterator>::value_type*>
{
public:

    typedef std::iterator<
            typename std::iterator_traits<Iterator>::iterator_category,
            typename std::iterator_traits<Iterator>::value_type*> Base;

    pointer_iterator(Iterator it) : m_it(it) { }

    pointer_iterator& operator+=(typename Base::difference_type n) { m_it += n; return *this; }
    pointer_iterator& operator-=(typename Base::difference_type n) { return operator+=(-n); }

private:

    Iterator m_it;
};

template <typename Iterator>
pointer_iterator<Iterator> operator+(
    pointer_iterator<Iterator> it,
    typename std::iterator_traits<pointer_iterator<Iterator>>::difference_type n)
{
    pointer_iterator<Iterator> nit = it;
    nit += n;
    return nit;
}

template <typename Iterator>
pointer_iterator<Iterator> pointer_it(Iterator it)
{
    return pointer_iterator<Iterator>(it);
}

BOOST_AUTO_TEST_CASE(PushTest)
{
    boost::container::stable_vector<open_element> elements;
    pointer_it(elements.begin());
    pointer_it(elements.end());
}
