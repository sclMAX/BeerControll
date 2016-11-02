#ifndef ETAPA
#define ETAPA
class Etapa{
        public:
        int inicio = 0; //Minuto de inicio 
        int duracion = 15; //Duracion de la etapa en minutos;
        bool manejaMacerador = true;
        int tempMacerador = 65; //Temperatura Macerador
        bool manejaLicor = true;
        int tempLicor = 78; //Temperatura Licor;
        bool recircula = false; //On/Off bomba de recirculado;
        bool alarma = true;
};
#endif //ETAPA