#ifndef IWIRELESS
#define IWIRELESS

namespace Wireless
{
class IWirelessCommunication
{
public:
 virtual void StartAntenna( ) = 0;
 virtual void StopAntenna( ) = 0;
 virtual bool SendMessageTo( const uint8_t* msg ) = 0;
 virtual void BroadcastMessage( const uint8_t* msg ) = 0;
 virtual void DebugingMode( const bool on ) = 0;
 virtual const bool On( ) = 0;
};
}  // namespace Wireless

#endif  // WIRELESS
