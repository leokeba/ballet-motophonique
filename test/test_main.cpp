// Host test runner: ./motophonique_tests [filter]
#include "motor_cases.h"
#include "tree_cases.h"
#include "master_cases.h"

int main(int argc, char** argv) { return check::run(argc > 1 ? argv[1] : nullptr) ? 1 : 0; }
