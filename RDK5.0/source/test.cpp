#include <iostream>
#include <stack>
#include <array>

int main(int argc, char* argv[])
{
    
    std::stack<std::array<int, 2>> arrs;
    arrs.push({1, 2});
    arrs.push({3, 2});
    arrs.push({4, 2});
    arrs.push({5, 2});
    

        while (!arrs.empty())            
    {

        auto show = arrs.top();           
        arrs.pop();

        std::cout<< ' ' <<show[1]<<std::endl;

    }




}