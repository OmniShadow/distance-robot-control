#include <iostream>
#include <numeric>
#include <vector>
#include <Regolatore.hpp>
using namespace std;


void Regolatore::reset(){
    previous_inputs.resize(previous_inputs.size(), 0);
    previous_outputs.resize(previous_outputs.size(), 0);
}

float Regolatore::calculate_output(float input)
{
    // Aggiorna il vettore degli ingressi precedenti spostando tutti i valori di un posto in avanti
    for (int i = previous_inputs.size() - 1; i > 0; i--)
    {
        previous_inputs[i] = previous_inputs[i - 1];
    }
    // Il valore nella prima posizione corrisponde all'ingresso attuale
    previous_inputs[0] = input;
    // Il contributo dell'ingresso all'uscita è il prodotto scalare tra il vettore degli ingressi precedenti che comprende quello attuale
    // e i coefficienti relativi agli ingressi della funzione di trasferimento del regolatore
    float output_input_component = inner_product(previous_inputs.begin(), previous_inputs.end(), input_coefficients.begin(), 0.0);

    // Il contributo dell'uscita precedente all'uscita attuale è il prodotto scalare tra il vettore delle uscite precedenti
    // e i coefficienti relativi all'uscita della funzione di trasferimento del regolatore
    float output_output_component = inner_product(previous_outputs.begin(), previous_outputs.end(), output_coefficients.begin(), 0.0);

    
    for (int i = previous_outputs.size() - 1; i > 0; i--)
    {
        previous_outputs[i] = previous_outputs[i - 1];
    }
    previous_outputs[0] = output_input_component + output_output_component;

    
    return previous_outputs[0];
}

Regolatore::Regolatore(vector<float> output_coeff, vector<float> input_coeff)
{
    output_coefficients = output_coeff;
    input_coefficients = input_coeff;

    // Inizializza i vettori dei precedenti ingressi e uscite a zero
    previous_inputs.resize(output_coefficients.size(), 0);
    previous_outputs.resize(input_coefficients.size(), 0);

}

