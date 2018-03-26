#include <gtest/gtest.h> // googletest header file
#include "../file_coordinates.h"

/**
*  Files Test OK
*  1 10.4 20.4 34
*  2 0.1 0.4 0.0032
*
*/
#include <string>
#include <iostream>
#include <vector>
#include <cstring>

#define FILE_OK "test_ok.txt"
#define FILE_ERROR "test_error.txt"
#define FILE_CRITICAL "test_critical.txt"


using std::string;

const string s1_x = "10.4";
const string s1_y = "20.4";
const string s1_ang = "34";

const string s2_x = "0.1";
const string s2_y = "0.4";
const string s2_ang = "0.0032";

POINT p[2];

void init()
{

p[0].x = atof(s1_x.c_str());
p[0].y = atof(s1_y.c_str());
p[0].ang = atof(s1_ang.c_str());

p[1].x = atof(s2_x.c_str());
p[1].y = atof(s2_y.c_str());
p[1].ang = atof(s2_ang.c_str());

}


TEST(DoubleComp, CStrEqual) {
    init();
    FileCoordinates fCoordinates;
    std::vector<POINT> vec;
    std::string s(FILE_OK);
    fCoordinates.GetCoordinatesVectorFromFile(s, vec);
    ASSERT_DOUBLE_EQ(p[0].x, vec[0].x);
    ASSERT_DOUBLE_EQ(p[0].y, vec[0].y);
    ASSERT_DOUBLE_EQ(p[0].ang, vec[0].ang);
    ASSERT_DOUBLE_EQ(p[1].x, vec[1].x);
    ASSERT_DOUBLE_EQ(p[1].y, vec[1].y);
    ASSERT_DOUBLE_EQ(p[1].ang, vec[1].ang);
}

TEST(DoubleComp1, CStrNotEqual) {

    FileCoordinates fCoordinates;
    std::vector<POINT> vec;
    std::string s(FILE_ERROR);
    fCoordinates.GetCoordinatesVectorFromFile(s, vec);
    ASSERT_DOUBLE_EQ(p[0].x, vec[0].x);
    ASSERT_DOUBLE_EQ(p[0].y, vec[0].y);
    ASSERT_DOUBLE_EQ(p[0].ang, vec[0].ang);
    ASSERT_DOUBLE_EQ(p[1].x, vec[1].x);
    ASSERT_DOUBLE_EQ(p[1].y, vec[1].y);
    ASSERT_DOUBLE_EQ(p[1].ang, vec[1].ang);
}


TEST(DoubleComp2, CStrCriticalEqual) {
    FileCoordinates fCoordinates;
    std::vector<POINT> vec;
    std::string s(FILE_CRITICAL);
    fCoordinates.GetCoordinatesVectorFromFile(s, vec);
    ASSERT_DOUBLE_EQ(p[0].x, vec[0].x);
    ASSERT_DOUBLE_EQ(p[0].y, vec[0].y);
    ASSERT_DOUBLE_EQ(p[0].ang, vec[0].ang);
    ASSERT_DOUBLE_EQ(p[1].x, vec[1].x);
    ASSERT_DOUBLE_EQ(p[1].y, vec[1].y);
    ASSERT_DOUBLE_EQ(p[1].ang, vec[1].ang);
}