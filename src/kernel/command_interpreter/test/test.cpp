#include <gtest/gtest.h>
#include "command_parser.hpp"

#define exp_throw(s) EXPECT_THROW(command_parser::parse(s), ParseError)
#define exp_nothrow(s) EXPECT_NO_THROW(command_parser::parse(s))
#define parse_result(s) command_parser::parse(s)

TEST(parser_test, empty_string) {
    exp_throw("  ");
    exp_throw("");
    exp_throw("          ");
}

TEST(parser_test, invalid_commands) {
    exp_throw("random string");
    exp_throw("random string 3");
    exp_throw("absolutlyrandomstring");
}

TEST(parser_test, wrong_arguments_amount) {
    exp_throw("start abdfd 2");
    exp_throw("stop absdf 3");
    exp_throw("stop absdf 3 14 16");
    exp_throw("start 17839 32 534");
    exp_throw("stop 4414 13425 i43");
}

TEST(parser_test, valid_commands) {
    exp_nothrow("start scenario 1 2");
    exp_nothrow("stop scenario 2 3");
    exp_nothrow("stop abcdfals 3 4");
    exp_nothrow("start process 3 5");
    exp_nothrow("stop things777");
}

TEST(parser_test, valid_commands_with_extra_symbols) {
    exp_nothrow("   start     scenario    1  1 key value key2   value2");
    exp_nothrow("   stop  alias 12 14 kkL00Lkey vv=))alue");
    exp_nothrow("start   allliasss 23 13 jlj lj    ");
    exp_nothrow("  stop  state 15 32 ad(((fa a{}dsf f f g[]]; g");
}


TEST(parser_test, return_values) {
    core_msgs::CI_to_HLLGoal res = parse_result("stop AlIa3 1 1 KkEY valueee [skbdsh] \\{#ffffff} 117 64 LUV FOREVE blah blah777");

    EXPECT_EQ(res.command, command_parser::STOP);
    EXPECT_EQ(res.alias, "AlIa3");

    EXPECT_EQ(res.params[0].ID_Proc, 1);
    EXPECT_EQ(res.params[0].ID_TL, 1);
    EXPECT_EQ(res.params[0].key_value[0].key, "KkEY");
    EXPECT_EQ(res.params[0].key_value[0].value, "valueee");
    EXPECT_EQ(res.params[0].key_value[1].key, "[skbdsh]");
    EXPECT_EQ(res.params[0].key_value[1].value, "\\{#ffffff}");

    EXPECT_EQ(res.params[1].ID_Proc, 64);
    EXPECT_EQ(res.params[1].ID_TL, 117);
    EXPECT_EQ(res.params[1].key_value[0].key, "LUV");
    EXPECT_EQ(res.params[1].key_value[0].value, "FOREVE");
    EXPECT_EQ(res.params[1].key_value[1].key, "blah");
    EXPECT_EQ(res.params[1].key_value[1].value, "blah777");
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
