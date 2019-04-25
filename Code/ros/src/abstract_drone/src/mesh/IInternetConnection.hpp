#ifndef IINTERNETCONNECTION
#define IINTERNETCONNECTION
class IInternetConnection
{
public:
 virtual void connect( ) = 0;
 virtual void disconnect( ) = 0;
};

#endif  // IINTERNETCONNECTION
