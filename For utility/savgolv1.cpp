#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

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
    // Test the filter
    vector<double> data = {0.162182308193243,0.794284540683907,0.311215042044805,0.528533135506213,0.165648729499781,0.601981941401637,0.262971284540144,0.654079098476782,0.689214503140008,0.748151592823709};
    int window_size = 5;
    int polynomial_order = 2;
    int derivative_order = 2;
    vector<double> filtered_data = center_weighted_savitzky_golay_filter(data, window_size, polynomial_order, derivative_order);

    for (auto i : filtered_data) {
        cout << i << " ";
    }

    return 0;
}
