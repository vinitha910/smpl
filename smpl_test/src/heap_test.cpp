#include <stdio.h>
#include <iostream>
#include <type_traits>
#include <vector>

#define BOOST_TEST_MODULE HeapTest
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <boost/container/stable_vector.hpp>

#include <smpl/intrusive_heap.h>

#define LOGDEBUG 0
#if LOGDEBUG
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

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

    /// \name Iterator Requirements
    ///@{}
    typename Base::value_type operator*() { return &(*m_it); }
    const typename Base::value_type operator*() const { return &(*m_it); }

    pointer_iterator operator++(int) { pointer_iterator it(m_it); ++m_it; return it; }
    pointer_iterator& operator++() { ++m_it; return *this; }
    ///@}

    /// \name Random Access Iterator Requirements
    ///@{}
    pointer_iterator& operator+=(typename Base::difference_type n)
    {
        LOG("%s\n", __PRETTY_FUNCTION__);
        m_it += n;
        return *this;
    }
    pointer_iterator& operator-=(typename Base::difference_type n)
    {
        LOG("%s\n", __PRETTY_FUNCTION__);
        return operator+=(-n);
    }
    typename Base::difference_type operator-(pointer_iterator<Iterator> it)
    {
        LOG("%s\n", __PRETTY_FUNCTION__);
        return m_it - it.m_it;
    }
    ///@}

    bool operator==(pointer_iterator it) const { return it.m_it == m_it; }
    bool operator!=(pointer_iterator it) const { return it.m_it != m_it; }

private:

    Iterator m_it;
};

template <typename Iterator>
pointer_iterator<Iterator> operator+(
    pointer_iterator<Iterator> it,
    typename std::iterator_traits<pointer_iterator<Iterator>>::difference_type n)
{
    LOG("%s\n", __PRETTY_FUNCTION__);
    pointer_iterator<Iterator> nit = it;
    nit += n;
    return nit;
}

template <typename Iterator>
pointer_iterator<Iterator> operator-(
    pointer_iterator<Iterator> it,
    typename std::iterator_traits<pointer_iterator<Iterator>>::difference_type n)
{
    LOG("%s\n", __PRETTY_FUNCTION__);
    pointer_iterator<Iterator> nit = it;
    nit -= n;
    return nit;
}

template <typename Iterator>
pointer_iterator<Iterator> operator+(
    typename std::iterator_traits<pointer_iterator<Iterator>>::difference_type n,
    pointer_iterator<Iterator> it)
{
    LOG("%s\n", __PRETTY_FUNCTION__);
    pointer_iterator<Iterator> nit = it;
    nit += n;
    return nit;
}

template <typename Iterator>
pointer_iterator<Iterator> pointer_it(Iterator it)
{
    return pointer_iterator<Iterator>(it);
}

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

BOOST_AUTO_TEST_CASE(PointerIteratorTest)
{
    boost::container::stable_vector<open_element> elements;
    elements.push_back(open_element(8));
    elements.push_back(open_element(10));
    elements.push_back(open_element(4));
    elements.push_back(open_element(2));
    elements.push_back(open_element(12));

    heap_type h(pointer_it(elements.begin()), pointer_it(elements.end()));
    BOOST_CHECK(!h.empty());
    BOOST_CHECK(h.size() == 5);
}

BOOST_AUTO_TEST_CASE(PushTest)
{
    std::vector<open_element> elements;
    elements.push_back(open_element(8));
    elements.push_back(open_element(10));
    elements.push_back(open_element(4));
    elements.push_back(open_element(2));
    elements.push_back(open_element(12));

    heap_type h;
    h.push(&elements[0]);

    BOOST_CHECK(!h.empty());
    BOOST_CHECK(h.size() == 1);
    BOOST_CHECK(h.min() == &elements[0]);

    h.push(&elements[1]);
    BOOST_CHECK(!h.empty());
    BOOST_CHECK(h.size() == 2);
    BOOST_CHECK(h.min() == &elements[0]);

    h.push(&elements[2]);
    BOOST_CHECK(!h.empty());
    BOOST_CHECK(h.size() == 3);
    BOOST_CHECK(h.min() == &elements[2]);

    h.push(&elements[3]);
    BOOST_CHECK(!h.empty());
    BOOST_CHECK(h.size() == 4);
    BOOST_CHECK(h.min() == &elements[3]);

    h.push(&elements[4]);
    BOOST_CHECK(!h.empty());
    BOOST_CHECK(h.size() == 5);
    BOOST_CHECK(h.min() == &elements[3]);
}

BOOST_AUTO_TEST_CASE(ClearTest)
{
    std::vector<open_element> elements;
    elements.push_back(open_element(8));
    elements.push_back(open_element(10));
    elements.push_back(open_element(4));
    elements.push_back(open_element(2));
    elements.push_back(open_element(12));

    heap_type h(pointer_it(elements.begin()), pointer_it(elements.end()));
    h.clear();
    BOOST_CHECK(h.empty());
    BOOST_CHECK(h.size() == 0);
}

BOOST_AUTO_TEST_CASE(PopTest)
{
    std::vector<open_element> elements;
    elements.push_back(open_element(8));
    elements.push_back(open_element(10));
    elements.push_back(open_element(4));
    elements.push_back(open_element(2));
    elements.push_back(open_element(12));

    heap_type h(pointer_it(elements.begin()), pointer_it(elements.end()));
    h.pop();
    BOOST_CHECK(h.min() == &elements[2]);

    h.pop();
    BOOST_CHECK(h.min() == &elements[0]);

    h.pop();
    BOOST_CHECK(h.min() == &elements[1]);

    h.pop();
    BOOST_CHECK(h.min() == &elements[4]);

    h.pop();
    BOOST_CHECK(h.empty());
}

BOOST_AUTO_TEST_CASE(EraseTest)
{
    std::vector<open_element> elements;
    elements.push_back(open_element(8));
    elements.push_back(open_element(10));
    elements.push_back(open_element(4));
    elements.push_back(open_element(2));
    elements.push_back(open_element(12));

    heap_type h(pointer_it(elements.begin()), pointer_it(elements.end()));

    h.erase(&elements[3]);
    BOOST_CHECK(h.size() == 4);

    h.erase(&elements[0]);
    BOOST_CHECK(h.size() == 3);

    h.erase(&elements[2]);
    BOOST_CHECK(h.size() == 2);

    h.erase(&elements[1]);
    BOOST_CHECK(h.size() == 1);

    h.erase(&elements[4]);
    BOOST_CHECK(h.size() == 0);
    BOOST_CHECK(h.empty());
}

BOOST_AUTO_TEST_CASE(ContainsTest)
{
    std::vector<open_element> elements;
    elements.push_back(open_element(8));
    elements.push_back(open_element(10));
    elements.push_back(open_element(4));
    elements.push_back(open_element(2));
    elements.push_back(open_element(12));

    heap_type h;

    h.push(&elements[0]);
    BOOST_CHECK(h.contains(&elements[0]));

    h.push(&elements[1]);
    BOOST_CHECK(h.contains(&elements[1]));

    h.push(&elements[2]);
    BOOST_CHECK(h.contains(&elements[2]));

    h.push(&elements[3]);
    BOOST_CHECK(h.contains(&elements[3]));

    h.push(&elements[4]);
    BOOST_CHECK(h.contains(&elements[4]));

    h.erase(&elements[4]);
    BOOST_CHECK(!h.contains(&elements[4]));

    h.erase(&elements[3]);
    BOOST_CHECK(!h.contains(&elements[3]));

    h.erase(&elements[2]);
    BOOST_CHECK(!h.contains(&elements[2]));

    h.erase(&elements[1]);
    BOOST_CHECK(!h.contains(&elements[1]));

    h.erase(&elements[0]);
    BOOST_CHECK(!h.contains(&elements[0]));
}

BOOST_AUTO_TEST_CASE(MakeTest)
{
    struct state : sbpl::heap_element
    {
        int g;
        int h;

        state() = default;
        state(int g, int h) : g(g), h(h) { }
    };

    struct state_compare
    {
        state_compare(const int* w) : m_w(w) { }
        const int* m_w;

        bool operator()(const sbpl::heap_element* a, const sbpl::heap_element* b)
        {
            return static_cast<const state*>(a)->g + (*m_w) * static_cast<const state*>(a)->h <
                    static_cast<const state*>(b)->g + (*m_w) * static_cast<const state*>(b)->h;
        }
    };

    typedef sbpl::intrusive_heap<state_compare> state_heap;

    std::vector<state> elements;
    elements.push_back(state(8, 2));    // 10
    elements.push_back(state(10, 1));   // 11
    elements.push_back(state(4, 4));    // 8
    elements.push_back(state(2, 3));    // 5
    elements.push_back(state(12, 0));   // 12

    int w = 1;
    state_compare comp(&w);
    state_heap open(comp, pointer_it(elements.begin()), pointer_it(elements.end()));

    BOOST_CHECK(open.min() == &elements[3]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[2]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[0]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[1]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[4]);
    open.pop();
    BOOST_CHECK(open.empty());

    open = state_heap(comp, pointer_it(elements.begin()), pointer_it(elements.end()));

    w = 3;
    // elements[0] =  8 + 3 * 2 = 14
    // elements[1] = 10 + 3 * 1 = 13
    // elements[2] =  4 + 3 * 4 = 16
    // elements[3] =  2 + 3 * 3 = 11
    // elements[4] = 12 + 3 * 0 = 12
    open.make();

//    int last_key = ((state*)open.min())->g + w * ((state*)open.min())->h;
//    printf("%p\n", open.min());
//    while (!open.empty()) {
//        open.pop();
//        printf("%p\n", open.min());
//        int min_key = ((state*)open.min())->g + w * ((state*)open.min())->h;
//        BOOST_CHECK(min_key >= last_key);
//        printf("%d >= %d\n", min_key, last_key);
//        last_key = min_key;
//    }

    BOOST_CHECK(open.min() == &elements[3]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[4]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[1]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[0]);
    open.pop();
    BOOST_CHECK(open.min() == &elements[2]);
    open.pop();
    BOOST_CHECK(open.empty());
}

BOOST_AUTO_TEST_CASE(InvalidIteratorTest)
{
    std::vector<int> ints;
    *ints.end();
}
