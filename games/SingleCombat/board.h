#pragma once
#include <vector>
#include <memory>
#include "block.h"
#include "userinput.h"


class Board
{
public:
    Board();
    void update();
    bool checkIfEmpty();
    void changeDestination(int column);
    void changeActiveColumn(double Xcoord);
private:
    //vector of vectors. A grid of blocks
    typedef std::shared_ptr<Block> pointer;
    typedef std::vector<pointer> blocks;
    std::vector<blocks> grid;

    //pointer to the current block
    std::shared_ptr<Block> currentBlock;

    const double boardLeftEdge{144};
    const double boardRightEdge{432};
    const double boardTopEdge{18};
    const double boardBottomEdge{882};
    double leftEdge;
    double rightEdge;
    int activeColumn;
    int activeRow;
    double currentDestination;

    UserInput userInput;
};