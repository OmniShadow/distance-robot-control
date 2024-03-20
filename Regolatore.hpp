#ifndef REGOLATORE_HPP
#define REGOLATORE_HPP
#include <vector>
using namespace std;

class Regolatore
{
private:
    vector<float> output_coefficients;
    vector<float> input_coefficients;
    vector<float> previous_inputs;
    vector<float> previous_outputs;

public:
    Regolatore(vector<float> output_coeff, vector<float> input_coeff);
    float calculate_output(float input);
    void reset();
};
#endif