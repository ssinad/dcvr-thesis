#ifndef __PENALTIES__
#define __PENALTIES__
#include <vector>
using penalty_t = double ;
using reward_t = penalty_t ;
using Penalties = std::vector<penalty_t>;
using Rewards = Penalties;
const reward_t REWARD_EPSILON = 1e-6;

#endif