#include <stdio.h>
#include <vector>

#include <smpl/intrusive_heap.h>

int main(int argc, char* argv[])
{
    struct open_element : sbpl::heap_element
    {
        int priority;

        open_element(int p = int()) : priority(p) { }
    };


    auto open_element_comp = [](
        const sbpl::heap_element* a,
        const sbpl::heap_element* b)
    {
        return static_cast<const open_element*>(a)->priority <
                static_cast<const open_element*>(b)->priority;
    };

    std::vector<open_element> elements = { 8, 10, 4, 2, 12 };

    std::vector<sbpl::heap_element*> element_ptrs = {
        &elements[0],
        &elements[1],
        &elements[2],
        &elements[3],
        &elements[4],
    };

    typedef decltype(open_element_comp) comp_type;
    sbpl::intrusive_heap<comp_type> open_list(open_element_comp, element_ptrs);

    for (sbpl::heap_element* e : open_list) {
        printf("%d\n", static_cast<open_element*>(e)->priority);
    }
    fflush(stdout);

    return 0;
}
