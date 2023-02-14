#include <iostream>
#include <fstream> 
#include<vector>
#define _USE_MATH_DEFINES
#include<math.h>
#include<string>

//Constantes

/*
ANGLE_X      = 80.0 #50.0
ANGLE_Y      = 30.0 #18.75

THETA_H = math.pi * ANGLE_X / 180.0
ALPHA_H = (math.pi - THETA_H) / 2

THETA_V = math.pi * ANGLE_Y / 180.0
ALPHA_V = 2 * math.pi - (THETA_V / 2)
*/
#define ANGLE_X 80.0
#define ANGLE_Y 30.0
#define HEIGHT 60
#define WIDTH 160
const float THETA_H = M_PI*ANGLE_X/180.0;   //pi * ANGLE_X / 180.0
const float ALPHA_H = (M_PI - THETA_H) / 2;
const float THETA_V = M_PI*ANGLE_Y/180.0;   //pi * ANGLE_X / 180.0
const float ALPHA_V = 2*M_PI - (THETA_V) / 2;
const int frameSize = 160*60*2*2;
//Mas constantes
#define VALUE_LIMIT_VALID_PIXEL 16000
#define VALUE_LOW_AMPLITUDE     16001
#define VALUE_ADC_OVERFLOW      16002
#define VALUE_SATURATION        16003

using namespace std;
int main(int argc,char** argv){
    //El formato de entrada va a ser el archivo donde estan todos los datos
    //El tamaño del archivo debera ser de 38400 bytes (es el tamaño que ocupa un frame de distancia+amplitud)
    uint8_t frame[frameSize] = {0};
    char c;
    int i = 0;
    string path = string(argv[1]);
    ifstream file(path);
    while (file.get(c)){
        //Iterar cada caracter del archivo
        frame[i] = c;
        i++;
    }
    file.close();
    if (i!=frameSize){
        //Ocurrio un error

    }
    //El objeto donde se guarda toda la informacion
    Frame F = Frame(HEIGHT,WIDTH,vector<float>(frameSize),vector<uint16_t>(frameSize),vector<uint16_t>(frameSize),vector<float>(frameSize),vector<point3d>(frameSize),frame);
    //Procesado
    //Iterar los puntos
    for (int i=0;i<=frameSize;i+=4){
        int x = i % 160;
        int y = int(i/ 160);
        uint8_t saturated_mask_v = 0;
        //Los primeros 2 bytes son la distancia en formato uint16
        uint16_t distMM = (frame[i] << 8) || frame[i+i];
        uint16_t amplitud = (frame[i+2] << 8) || frame[i+3];
        float X = NULL;
        float Y = NULL;
        float Z = NULL;
        if (distMM < VALUE_LIMIT_VALID_PIXEL || distMM == VALUE_ADC_OVERFLOW || distMM == VALUE_SATURATION){
            //Saturacion
            if (distMM == VALUE_ADC_OVERFLOW || distMM == VALUE_SATURATION)saturated_mask_v = 255;
            else{
                //TODO: colorear los puntos r,g,b, por ahora solo seran 0
                float gamma_i_h = ALPHA_H + x * (THETA_H / WIDTH);
                float gamma_i_v = ALPHA_V + y * (THETA_V / HEIGHT);

                Z = abs(0.001 * distMM * sin(gamma_i_h));
                Z = abs(Z * cos(gamma_i_v));

                X = Z / tan(gamma_i_h);
                Y = -1 * Z * tan(gamma_i_v);
            }
            point3d P;
            P.X = X;P.Y = Y; P.Z = Z;
            P.R = 0;P.G=0;P.B=0;
            F.data_depth[i] = Z;
            F.points_3d[i] = P;
            F.data_amplitude[i] = amplitud;
            F.data_depth_rgb[i] = 0;


        }


    }

}
struct point3d{
    float X;
    float Y;
    float Z;
    float R;
    float G;
    float B;
};
class Frame{
    public:
    int height;
    int width;
    std::vector<float> data_depth;
    std::vector<uint16_t> data_depth_rgb;
    std::vector<uint16_t> data_grayscale;
    std::vector<float>data_amplitude;
    std::vector<point3d> points_3d;
    uint8_t* data;
    Frame(){};
    Frame(int _height,int _width, std::vector<float> _data_depth, std::vector<uint16_t> _data_depth_rgb, std::vector<uint16_t> _data_grayscale, std::vector<float>_data_amplitude, std::vector<point3d> _points_3d, uint8_t* _data){
        height = _height;
        width = _width;
        data_depth = _data_depth;
        data_depth_rgb = _data_depth_rgb;
        data_grayscale = _data_grayscale;
        data_amplitude = _data_amplitude;
        points_3d = _points_3d;
        data = _data;

    }
};