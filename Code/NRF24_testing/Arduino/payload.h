// file payload.h
#ifndef __VIRIDI_NEXU_PAYLOAD__
#define __VIRIDI_NEXU_PAYLOAD__
 
enum PayloadType {
    HERBA, METEO
};
 
typedef uint8_t vn_payload_type;
typedef uint8_t vn_payload_version;
 
typedef struct {
        int16_t humidity;
        int16_t temperature;
        int16_t pressure;
        int16_t altitude;
        int16_t luminosity;
} vn_meteo_t;
 
typedef struct {
        int16_t moisture;
        int16_t temperature;
} vn_plant_t;
 
struct Payload {
        vn_payload_type type;
        vn_payload_version version;
 
        union {
            vn_meteo_t meteo;
            vn_plant_t plant;
        } data;
};
 
