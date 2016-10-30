#include <stdio.h>
#include <fstream>
#include <iostream>

//#include <ros/ros.h>
#include <smpl/csv_parser.h>

int main(int argc, char* argv[])
{
//    ros::init(argc, argv, "csv_parser_test");
//    ros::NodeHandle nh;

    if (argc < 2) {
        printf("Usage: csv_parser_test <path>\n");
        return 0;
    }
    std::ifstream ifs(argv[1]);

    sbpl::CSVParser parser;
    if (!parser.parseStream(ifs)) {
        fprintf(stderr, "failed to parse %s\n", argv[0]);
        return 1;
    }

    for (size_t j = 0; j < parser.fields().size() / parser.record_size(); ++j) {
        for (size_t i = 0; i < parser.record_size(); ++i) {
            std::cout << parser.fields()[j * parser.record_size() + i] << ", ";
        }
        std::cout << '\n';
    }

    return 0;
}
