#ifndef IINTERNETCONNECTION
#define IINTERNETCONNECTION
namespace Communication
{
class IInternetConnection
{
public:
 virtual void connect( ) = 0;
 virtual void disconnect( ) = 0;
};
}  // namespace Communication

#endif  // IINTERNETCONNECTION
