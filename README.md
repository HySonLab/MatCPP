# Matheuristics-CPPLC
 
C:\Program Files\IBM\ILOG\CPLEX_Studio127\cplex\include;
C:\Program Files\IBM\ILOG\CPLEX_Studio127\concert\include

C:\Program Files\IBM\ILOG\CPLEX_Studio127\cplex\lib\x64_windows_vs2015\stat_mda;
C:\Program Files\IBM\ILOG\CPLEX_Studio127\concert\lib\x64_windows_vs2015\stat_mda

cplex1270.lib;
concert.lib;
ilocplex.lib

https://adam-rumpf.github.io/documents/cplex_in_cpp.pdf

# Do this first, I have no idea why
'''
cmake -DCMAKE_CXX_COMPILER=cl -DCMAKE_C_COMPILER=cl -G "Visual Studio 17 2022" -S . -B .\build
cmake --build build --config Release
'''