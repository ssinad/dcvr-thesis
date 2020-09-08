#ifndef __BASIS_EVENT__
#define __BASIS_EVENT__
enum BasisEventType
{
    Entry,
    Exit
};

struct BasisEvent
{
    BasisEventType event_type;
    int iteration;
    int variable;
};
#endif