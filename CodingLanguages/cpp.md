# C++

## Stream I/O and Local Variables

- Printing to standard output and taking in input from standard input can be done using two syntaxes
  - Requires you to use **#include \<iostream>**
  - **std::cout << "Hello!" << "\n";** - You can send multiple characters/values at a time
  - **std::cin >> \<someVariable>**
- Namespaces can also be very useful for repetitive code. The **using** keyword allows you to avoid repeatedly typing the full namespace qualifier.
  - Can do **using namespace std;** for all std namespace objects
  - Can also do **using std::cout** for a specific namespace object
