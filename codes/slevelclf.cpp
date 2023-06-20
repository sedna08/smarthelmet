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
#include <cmath>
#include <algorithm>
#include <numeric>
#include <array>
#include <functional>
#include <iterator>

#include </usr/local/include/fdeep/fdeep.hpp>

using namespace std;

vector<double> center_weighted_savitzky_golay_filter(vector<double> data, int window_size, int polynomial_order, int derivative_order) {
    vector<double> filtered_data;
    int half_window = (window_size - 1) / 2;
    int n = data.size();
    double sum_weights = 0;

    // Calculate weights for the filter
    vector<double> weights(window_size);
    for (int i = -half_window; i <= half_window; i++) {
        double weight = 0;
        for (int j = 0; j <= polynomial_order; j++) {
            weight += pow(i, j);
        }
        weight = abs(weight);
        weights[i + half_window] = weight;
        sum_weights += weight;
    }

    // Apply the filter
    for (int i = 0; i < n; i++) {
        double filtered_value = 0;
        for (int j = -half_window; j <= half_window; j++) {
            int index = i + j;
            if (index < 0) {
                index = 0;
            }
            if (index >= n) {
                index = n - 1;
            }
            filtered_value += weights[j + half_window] * data[index];
        }
        filtered_value /= sum_weights;
        filtered_data.push_back(filtered_value);
    }

    return filtered_data;
}

int main() {

    vector<double> eegBuffer[8];
    vector<double> imuBufferTemp[2];
    vector<double> imuBuffer[2];
    vector<double> cvBuffer[1];

    // Open file for reading
    ifstream infile("data1.txt"); // not needed for model integration

    // Load converted model (JSON)
    const auto mymodel = fdeep::load_model("best_model(single-levelv1).json");

    double val;
    double dataArray[15];

    // Loop not needed for model integration if not going to use txt file instead make loop to push_back values from the output of each modules
    // Use loop for reading txt file if output is recorded
    for (int i = 0; i < 900; i++) {
        // Read line from file
        string line;
        getline(infile, line);

        // Create stringstream from line
        stringstream ss(line);

        int j = 0;
        // Read double values from stringstream
        while (ss >> val) {
            // printf("%lf\n",val);
            dataArray[j] = val;
            j++;
        }

        eegBuffer[0].push_back(dataArray[3]); // insert Delta Band to data vector
        eegBuffer[1].push_back(dataArray[4]); // insert Theta Band to data vector
        eegBuffer[2].push_back(dataArray[5]); // insert Low-alpha Band to data vector
        eegBuffer[3].push_back(dataArray[6]); // insert High-alpha Band to data vector
        eegBuffer[4].push_back(dataArray[7]); // insert Low-beta Band to data vector
        eegBuffer[5].push_back(dataArray[8]); // insert High-beta Band to data vector
        eegBuffer[6].push_back(dataArray[9]); // insert Low-gamma Band to data vector
        eegBuffer[7].push_back(dataArray[10]); // insert Mid-gamma Band to data vector

        // Inserting IMU data to imuBufferTemp to be processed later before proceeding to model prediction
        imuBufferTemp[0].push_back(dataArray[12]); // insert fittedRoll to data vector
        imuBufferTemp[1].push_back(dataArray[13]); // insert fittedPitch Band to data vector

        // Inserting PERCLOS data to cvBuffer for Model Prediction
        cvBuffer[0].push_back(dataArray[14]); // insert PERCLOS to data vector
    }

    // Close file
    infile.close();

    // TODO
    /*
    // Print eegBuffer first 10 values
    printf("EEG VALUES\n");
    for (int i = 0; i < 8; i++) {
        int j = 0;
        for (double v : eegBuffer[i]) {
            if (j < 100) {
                cout << v << " ";
                j++;
            }
        }
        cout << endl;
    }
    cout << endl;

    // Print imuBufferTemp first 10 values
    printf("IMU VALUES (PRE SGOLAY)\n");
    for (int i = 0; i < 2; i++) {
        int j = 0;
        for (double v : imuBufferTemp[i]) {
            if (j < 100) {
                cout << v << " ";
                j++;
            }
        }
        cout << endl;
    }
    cout << endl;

    // Print cvBuffer first 10 values
    printf("CV VALUES\n");
    for (int i = 0; i < 1; i++) {
        int j = 0;
        for (double v : cvBuffer[i]) {
            if (j < 100) {
                cout << v << " ";
                j++;
            }
        }
        cout << endl;
    }
    cout << endl;
    */

    int window_size = 51;
    int polynomial_order = 2;
    int derivative_order = 0;

    // SGolay Filter for Roll
    vector<double> sgolayRoll = center_weighted_savitzky_golay_filter(imuBufferTemp[0], window_size, polynomial_order, derivative_order);
    // SGolay Filter for Pitch
    vector<double> sgolayPitch = center_weighted_savitzky_golay_filter(imuBufferTemp[1], window_size, polynomial_order, derivative_order);

    // getting Derivative of sgolayRoll and sgolayPitch
    vector<double> dysgolayRoll(sgolayRoll.size(), 0.0);
    vector<double> dysgolayPitch(sgolayPitch.size(), 0.0);

    // diff() function in matlab
    std::adjacent_difference(std::begin(sgolayRoll), std::end(sgolayRoll), std::begin(dysgolayRoll));
    std::adjacent_difference(std::begin(sgolayPitch), std::end(sgolayPitch), std::begin(dysgolayPitch));

    // imuBuffer[0].at(0) = 0;
    // imuBuffer[1].at(0) = 0;
    imuBuffer[0] = dysgolayRoll;
    imuBuffer[1] = dysgolayPitch;
    imuBuffer[0][0] = 0.0;
    imuBuffer[1][0] = 0.0;

    // TODO
    /* Code here for evaluating prediction
     * Use eegBuffer vector array for eeg classifier
     * Use imuBuffer vector array for imu classifier
     * Use cvBuffer vector array for cv classifier
     */

    // Fill an fdeep::tensor with values, e.g., from an std::vector<float>
    // Convert double to float
    const vector<float> const_eegBuffer_0(eegBuffer[0].begin(), eegBuffer[0].end());
    const vector<float> const_eegBuffer_1(eegBuffer[1].begin(), eegBuffer[1].end());
    const vector<float> const_eegBuffer_2(eegBuffer[2].begin(), eegBuffer[2].end());
    const vector<float> const_eegBuffer_3(eegBuffer[3].begin(), eegBuffer[3].end());
    const vector<float> const_eegBuffer_4(eegBuffer[4].begin(), eegBuffer[4].end());
    const vector<float> const_eegBuffer_5(eegBuffer[5].begin(), eegBuffer[5].end());
    const vector<float> const_eegBuffer_6(eegBuffer[6].begin(), eegBuffer[6].end());
    const vector<float> const_eegBuffer_7(eegBuffer[7].begin(), eegBuffer[7].end());
    const vector<float> const_imuBuffer_0(imuBuffer[0].begin(), imuBuffer[0].end()); // IMU
    const vector<float> const_imuBuffer_1(imuBuffer[1].begin(), imuBuffer[1].end()); // IMU
    const vector<float> const_cvBuffer_0(cvBuffer[0].begin(), cvBuffer[0].end()); // CV

    // Initialize a const 900x11 vector
    const vector<vector<float>> v = { 
        const_eegBuffer_0,
        const_eegBuffer_1,
        const_eegBuffer_2,
        const_eegBuffer_3,
        const_eegBuffer_4,
        const_eegBuffer_5,
        const_eegBuffer_6,
        const_eegBuffer_7,
        const_imuBuffer_0,
        const_imuBuffer_1,
        const_cvBuffer_0
    };

    // Create a 900x11 tensor using the initialized 900x11 vector
    // Create fdeep::tensor with its own memory
    const int tensor_channels = 1;
    const int tensor_rows = 900;
    const int tensor_cols = 11;
    fdeep::tensor_shape tensor_shape(tensor_rows, tensor_cols);
    fdeep::tensor t(tensor_shape, 0.0f);

    for (int i = 0; i < tensor_rows; i++) 
    {
        for (int j = 0; j < tensor_cols; j++) 
        {
            t.set(fdeep::tensor_pos(i, j), v[j][i]);
        }
    }

    // Step X: Predict the model
    try
    {
        // Step X.x: Uncomment this section to show probability of classes
        // auto result = mymodel.predict({ t }); // predict class
        // std::cout << fdeep::show_tensors(result) << std::endl; // print the tensor

        // Step X.x (Optional): Uncomment this section to show predicted class
        //   - This section outputs 0 and 1
        //       - 0 = awake, 1 = drowsy
        const auto result = mymodel.predict_class({ t }); // predict class
        if (result == 1)
        {
            std::cout << "Drowsy" << std::endl;

        }
        else
        {
            std::cout << "Awake" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << '\n';
    }

    // TODO
    /*
    // Print imuBufferTemp first 10 values
    printf("IMU VALUES (POST SGOLAY)\n");
    for (int i = 0; i < 2; i++) {
        int j = 0;
        for (double v : imuBuffer[i]) {
            if (j < 100) {
                cout << v << " ";
                j++;
            }
        }
        cout << endl;
    }
    cout << endl;
    */

    return 0;
}
