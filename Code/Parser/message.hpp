#include <inttypes.h>
#include <string>

enum  messagetype : uint8_t {NOTDEFINED = 0 , LOCATION, SIGNON};

class message
{
public:
    message(uint8_t _ID, uint8_t _messagetype);
    ~message();
    virtual void toPayload(char* payload) = 0;
    virtual std::string toString() = 0;
    void CopyFromCharArray(char* value, uint16_t size, char* arr, uint16_t start);
    
protected:
    void CopyToCharArray(char* value, uint16_t size, char* arr, uint16_t start);
    uint8_t ID;
    uint8_t type;

};

class locationMessage : public message
{
public:
    locationMessage(uint8_t _ID, float latitude, float longitude, int16_t height, uint32_t timeSincePosix);
     locationMessage(uint8_t _ID, char* payload);
    ~locationMessage();
    std::string toString();
    void toPayload(char* payload);
private:
    float latitude, longitude;
    int16_t height;
    uint32_t timeSincePosix;
};

