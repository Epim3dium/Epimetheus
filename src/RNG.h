#ifndef GMX_RNG_H
#include <time.h>
#include <chrono>
#include <random>

/* used to create random numbers using random int distribution
 * * uses hashed time as seed value by default
 * * seed can be specified using Seed() function 
 * * use Random() to get float 0 -> 1 or specify your range
 */
class RNG {
public:
    typedef std::mt19937 rng_type;
    rng_type s_rng;
private:
    std::uniform_int_distribution<rng_type::result_type> s_udist;
public:
	void Seed(unsigned int seed_val) {
		rng_type::result_type const seedval = static_cast<uint32_t>(seed_val); // get this from somewhere
		s_rng.seed(seedval);
	}
    //returns float in the range from 0 to 1 
	float Random() {
		rng_type::result_type random_number = s_udist(s_rng);
		return (float)random_number / (float)0xFFFFFFFF;
	}
    //specify lowest value to be generated and high value which are bounds
    template<class T>
    T Random(T low, T high) {
        T diff = high - low;
        return low + (T)((float)diff * Random());
    }
	RNG(float seed = 0) {
        if(seed == 0) {
            uint32_t t = static_cast<uint32_t>( time(nullptr) );
            std::hash<uint32_t> hasher; size_t hashed=hasher(t);
            this->Seed( static_cast<uint32_t>(hashed) );
        } else
            this->Seed(seed);
    }
};

#define GMX_RNG_H
#endif
