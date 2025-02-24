# MatCPP: Efficient Matheuristics for Solving the Chinese Postman Problem with Load Constraints

## Contributors
- Thieu-Khang Nguyen  
- Truong-Son Hy  
- Thu-Huong Dang  

## Prerequisites
- [CMake](https://cmake.org/download/) (version 3.10 or higher)  
- [Visual Studio 2022](https://visualstudio.microsoft.com/vs/) with C++ development tools  
- [IBM ILOG CPLEX Studio 12.10](https://www.ibm.com/products/ilog-cplex-optimization-studio)  

Ensure that CPLEX and Concert include and library directories are correctly set in the `CMakeLists.txt`.

## Building the Project

1. **Clone the repository:**
   ```
   git clone https://github.com/HySonLab/MatCPP.git
   cd MatCPP
   ```

2. **Configure the project with CMake:**
    ```
    cmake -DCMAKE_CXX_COMPILER=cl -DCMAKE_C_COMPILER=cl -G "Visual Studio 17 2022" -S . -B ./build
    ```

3. **Build the project:**
    ```
    cmake --build ./build --config Release
    ```

## Running the Project
    ```
    ./build/Release/main.exe
    ```

## Notes

- **CPLEX Setup:**  
  Ensure that the CPLEX license and environment variables are properly configured before running the executable.

- **Linking Issues:**  
  If you encounter linking errors, verify that the paths in `include_directories` and `link_directories` within the `CMakeLists.txt` are correct.