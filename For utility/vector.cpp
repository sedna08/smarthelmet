#include <stdio.h>
#include <string.h>
#include <string>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <sstream>

using namespace std;

int main() {
    // Open file for reading
    ifstream infile("training2_AData.txt"); // not needed for model integration

    
    double val;
    vector<double> values[10]; // change array size to 900 

    // loop not needed for model integration if not going to use txt file instead make loop to push_back values from the output of each modules
    // use loop for reading txt file if output is recorded
    for(int i = 0; i < 10; i++) {
        // Read line from file
        string line;
        getline(infile, line);

        // Create stringstream from line
        stringstream ss(line);

        // Read double values from stringstream
        while (ss >> val) {
            values[i].push_back(val);
        }
    }

    // print loops not needed
    // Print all values
    for(int i = 0; i < 10; i++) {
        for (double v : values[i]) {
            cout << v << " ";
        }
        cout << endl;
    }
    cout << endl;

    // Print a particular column only
    for(int i = 0; i < 10; i++) {
        cout << values[i][5] << " ";
        cout << endl;
    }

    // Close file
    infile.close();

    return 0;
}

