#ifndef  EPI_PRIMITIVE_WRAPPER
#define  EPI_PRIMITIVE_WRAPPER
#include <type_traits>
namespace epi {
    template<class PrimitiveType>
    struct PrimitiveWrapper {
        static_assert(std::is_fundamental<PrimitiveType>::value);
    private:
        PrimitiveType val;
    public:
        PrimitiveType& operator()() {
            return val;
        }
        const PrimitiveType& operator()() const {
            return val;
        }
        operator PrimitiveType() {
            return val;
        }
        
        PrimitiveWrapper& operator=(PrimitiveType v) {
            val = v;
            return *this;
        }
        PrimitiveWrapper( PrimitiveType init_val = {}) : val(init_val) {} 
        PrimitiveWrapper( const PrimitiveWrapper<PrimitiveType>&) = default;
    };

}
#endif  //EPI_PRIMITIVE_WRAPPER
