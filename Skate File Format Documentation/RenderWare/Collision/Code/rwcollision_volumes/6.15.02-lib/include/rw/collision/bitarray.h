// (c) Electronic Arts. All Rights Reserved.

/**
 \file rw/collision/bitarray.h

 \brief utility class for a vector of bits.
*/

#ifndef RW_COLLISION_BITARRAY_H
#define RW_COLLISION_BITARRAY_H


// ***********************************************************************************************************
// Includes

#include "rw/collision/common.h"
#include <string.h>                 // for memcpy()

namespace rw
{
namespace collision
{

// ***********************************************************************************************************
// Defines

// ***********************************************************************************************************
// Forward Declarations

// ***********************************************************************************************************
// Typedefs

// ***********************************************************************************************************
// Enums + Consts

// ***********************************************************************************************************
// Global Variables

// ***********************************************************************************************************
// External Functions Prototypes

// ***********************************************************************************************************
// Structs + Unions + Classes

// ***********************************************************************************************************
//                                                BitArray Class Definition
// ***********************************************************************************************************

/**
A utility class for storing an array of bits (or flags).
The bits are stored in words.

Bits can be accessed through operator[] and modified via Set/Unset.
Individual bits may also be accessed through BitArray::Iterator and BitArray::ConstIterator.

Underlying words may be accessed directly through BitArray::WordIterator and BitArray::ConstWordIterator

The position of a bit referenced by an iterator is obtained through BitArray::GetIndex.

\importlib rwbittable
*/
class BitArray
{
private:
    // base class for all iterators
    class IteratorBase;

public:

    typedef uint32_t WordType;

    class ConstIterator;
    class Iterator;

    typedef const WordType* ConstWordIterator;
    typedef WordType*       WordIterator;

    // constructor and destructor
    BitArray();
    ~BitArray();

    // initialise the bit array from a block of memory
    void Initialize(WordType *mem, uint32_t sizeInBits);

    // get the number of bits in the bit array
    uint32_t GetSize() const;

    // return true if bit i is set
    RwpBool operator[](uint32_t i) const;
    // return true if bit i is set, throw an exception if i is out of bounds
    RwpBool at(uint32_t i) const;

    // set bit i
    void Set(uint32_t i);
    // unset bit i
    void Unset(uint32_t i);

    // set all bits in the array
    void SetAll();
    // unset all bits in the array
    void UnsetAll();

    // return begin and end iterators
    ConstIterator Begin() const;
    Iterator      Begin();
    ConstIterator End() const;
    Iterator      End();

    bool FirstOne(uint32_t & index);
    bool FirstZero(uint32_t & index);

    // get an iterator referencing the indexed bit
    ConstIterator GetIterator(uint32_t index) const;
    Iterator GetIterator(uint32_t index);

    // return the index of a bit referenced by an iterator
    uint32_t GetIndex(const IteratorBase &iter) const;

    // return begin and end iterators to the words used by the array
    ConstWordIterator WordBegin() const;
    WordIterator      WordBegin();
    ConstWordIterator WordEnd() const;
    WordIterator      WordEnd();

    template <class T>
    void IterateOnes(T & Handler, uint32_t startWord=0, uint32_t wordIncrement=1);

    template <class T>
    void IterateZeros(T & Handler, uint32_t startWord=0, uint32_t wordIncrement=1);

    void CopyFrom(const BitArray & other);

    void Not();
    void And(const BitArray & other);
    void Or(const BitArray & other);

    static uint32_t GetArraySize(uint32_t numBits);

private:
    WordType *m_data;
    uint32_t  m_sizeInBits;
    uint32_t  m_sizeInWords;

protected:
    bool AdvanceToOne_Branchless(WordType & word, uint32_t & index);
    bool AdvanceToZero_Branchless(WordType & word, uint32_t & index);

    bool AdvanceToOne_Branching(WordType & word, uint32_t & index);
    bool AdvanceToZero_Branching(WordType & word, uint32_t & index);
    
    bool AdvanceToOne(WordType & word, uint32_t & index);
    bool AdvanceToZero(WordType & word, uint32_t & index);

    enum MiscConstants
    {
        /**
        
        The number of bits stored in each word of the array.
        This must be a power of 2.
        \cond dev
        */
        BITS_PER_WORD = 32,
        /**
        \endcond
        */

        /**
        \cond dev
        The right shift required to obtain the word index from a bit index.
        This is the log2 of BITS_PER_WORD.
        */
        WORD_SHIFT = 5
        /**
        \endcond
        */

    };

};


// ***********************************************************************************************************
//                                                BitArray::IteratorBase Class Definition
// ***********************************************************************************************************

/**
Base class for all BitArray iterators.

\importlib rwbittable
*/
class BitArray::IteratorBase
{
public:
    IteratorBase();
    IteratorBase(WordType *word, uint32_t bitIndex);
    IteratorBase(const IteratorBase &other);
    const IteratorBase& operator=(const IteratorBase &other);

    // equality
    RwpBool operator==(const IteratorBase &other) const;
    // inequality
    RwpBool operator!=(const IteratorBase &other) const;
    // less than
    RwpBool operator<(const IteratorBase &other) const;
    // other operators
    RwpBool operator>(const IteratorBase &other) const;
    RwpBool operator>=(const IteratorBase &other) const;
    RwpBool operator<=(const IteratorBase &other) const;
    // get the value of the referenced bit
    RwpBool operator*() const;
    // find a one bit, search up to end
    RwpBool FindOne(const IteratorBase &end);
    // find a zero bit, search up to end
    RwpBool FindZero(const IteratorBase &end);

protected:
    // destructor
    ~IteratorBase();

    // move to the next bit
    void MoveToNextBit();
    void MoveForward(uint32_t distance);
    void MoveBackward(uint32_t distance);

    WordType *m_word;
    uint32_t  m_bitIndex;

private:
    friend class BitArray;   // for BitArray::GetIndex(IteratorBase&)
};


// ***********************************************************************************************************
//                                                BitArray::Iterator Class Definition
// ***********************************************************************************************************

/**
Iterator for iterating through individual bits in a BitArray.

\importlib rwbittable
*/
class BitArray::Iterator : public BitArray::IteratorBase
{
public:
    Iterator();
    Iterator(uint32_t *word, uint32_t bitIndex);
    Iterator(const Iterator &other);
    ~Iterator();

    const Iterator& operator=(const Iterator &other);

    Iterator& operator++();
    Iterator operator+(uint32_t distance) const;
    Iterator operator-(uint32_t distance) const;

    void Set() const;
    void Unset() const;
};


// ***********************************************************************************************************
//                                                BitArray::ConstIterator Class Definition
// ***********************************************************************************************************

/**
Const iterator for iterating through individual bits in a BitArray.

\importlib rwbittable
*/
class BitArray::ConstIterator : public BitArray::IteratorBase
{
public:
    ConstIterator();
    ConstIterator(WordType *word, uint32_t bitIndex);
    ConstIterator(const IteratorBase &other);
    ~ConstIterator();

    const ConstIterator& operator=(const ConstIterator &other);

    ConstIterator& operator++();
    ConstIterator operator+(uint32_t distance) const;
    ConstIterator operator-(uint32_t distance) const;
};



// ********************************************************************************************************
//    Inline methods


/**
Default constructor for an iterator.
*/
inline
BitArray::IteratorBase::IteratorBase()
{
}

/**
\cond dev
*/
inline
BitArray::IteratorBase::IteratorBase(BitArray::WordType *word, uint32_t bitIndex)
  : m_word(word),
    m_bitIndex(bitIndex)
{
}
/**
\endcond
*/


/**
\cond dev
*/
inline
BitArray::IteratorBase::IteratorBase(const BitArray::IteratorBase &other)
  : m_word(other.m_word),
    m_bitIndex(other.m_bitIndex)
{
}

/**
\endcond
*/


/**
\cond dev
*/
inline
BitArray::IteratorBase::~IteratorBase()
{
}
/**
\endcond
*/

/**
\cond dev
*/
inline const BitArray::IteratorBase&
BitArray::IteratorBase::operator=(const BitArray::IteratorBase &other)
{
    m_word = other.m_word;
    m_bitIndex = other.m_bitIndex;
    return (*this);
}
/**
\endcond
*/

/**
Compare two iterators for equality.
*/
inline RwpBool
BitArray::IteratorBase::operator==(const BitArray::IteratorBase &other) const
{
    return ((RwpBool)((m_word == other.m_word) && (m_bitIndex == other.m_bitIndex)));
}

/**
Compare two iterators for non-equality.
*/
inline RwpBool
BitArray::IteratorBase::operator!=(const BitArray::IteratorBase &other) const
{
    return ((RwpBool)(!operator==(other)));
}

/**
\cond dev
*/
inline void
BitArray::IteratorBase::MoveToNextBit()
{
    EA_ASSERT(m_bitIndex < BITS_PER_WORD);
    if (++m_bitIndex == BITS_PER_WORD)
    {
        m_bitIndex = 0;
        ++m_word;
    }
}
/**
\endcond
*/


/**
\cond dev
*/
inline void
BitArray::IteratorBase::MoveForward(uint32_t distance)
{
    EA_ASSERT(m_bitIndex < BITS_PER_WORD);
    EA_ASSERT_FORMATTED(int32_t(distance) >= 0, ("Iterator move distance %d must not be negative.", distance));
    m_word += distance >> WORD_SHIFT;
    m_bitIndex += distance & (BITS_PER_WORD - 1);
    if (m_bitIndex >= BITS_PER_WORD)
    {
        m_bitIndex -= BITS_PER_WORD;
        ++m_word;
    }
}
/**
\endcond
*/


/**
\cond dev
*/
inline void
BitArray::IteratorBase::MoveBackward(uint32_t distance)
{
    EA_ASSERT(m_bitIndex < BITS_PER_WORD);
    EA_ASSERT_FORMATTED(int32_t(distance) >= 0, ("Iterator move distance %d must not be negative.", distance));
    uint32_t i = distance & (BITS_PER_WORD - 1);
    m_word -= distance >> WORD_SHIFT;
    if (i > m_bitIndex)
    {
        m_bitIndex += BITS_PER_WORD - i;
        --m_word;
    }
    else
    {
        m_bitIndex -= i;
    }
}
/**
\endcond
*/



/**
Compare two iterators.
\return true if the first iterator is less than the second.
*/
inline RwpBool
BitArray::IteratorBase::operator<(const IteratorBase &other) const
{
    return ((RwpBool)(m_word < other.m_word || (m_word == other.m_word && m_bitIndex < other.m_bitIndex) ));
}


/**
Compare two iterators.
\return true if the first iterator is greater than the second.
*/
inline RwpBool
BitArray::IteratorBase::operator>(const IteratorBase &other) const
{
    return (other < *this);
}

/**
Compare two iterators.
\return true if the first iterator is greater than or equal to the second.
*/
inline RwpBool
BitArray::IteratorBase::operator>=(const IteratorBase &other) const
{
    return ((RwpBool)(!(*this < other)));
}

/**
Compare two iterators.
\return true if the first iterator is less than or equal to the second.
*/
inline RwpBool
BitArray::IteratorBase::operator<=(const IteratorBase &other) const
{
    return ((RwpBool)(!(other < *this)));
}


/**
Get the value of the referenced bit.
\return true if the bit is set, false if it is not set.
*/
inline RwpBool
BitArray::IteratorBase::operator*() const
{
    EA_ASSERT(m_bitIndex < BITS_PER_WORD);
    return ((*m_word >> m_bitIndex) & 1);
}


/**
While the iterator value is zero, advance it.  If end is reached, return false.
\param end highest position to which the iterator can be advanced.
\return true if one found, else false.
*/
inline RwpBool
BitArray::IteratorBase::FindOne(const IteratorBase &end)
{
    EA_ASSERT_MSG(m_bitIndex < BITS_PER_WORD, ("Iterator bit index not valid."));
    EA_ASSERT_MSG(!(end < *this), ("Iterator position is past end."));
    uint32_t bit = (uint32_t)(1 << m_bitIndex);

    // The main loop performs only a single bit test, and avoids checking the word index, for speed.
    while (!(*m_word & bit))
    {
        bit *= 2;
        if (++m_bitIndex == BITS_PER_WORD)
        {
            m_bitIndex = 0;
            bit = 1;
            do
            {
                if (++m_word > end.m_word)
                {
                    operator=(end);     // *this = end
                    return (FALSE);
                }
            }
            while (*m_word == 0);

            // This guards against the tricky (but common) case where the end bit is the first bit of
            // an unused word one-past the last used word.
            if (operator==(end))
            {
                return (FALSE);
            }
        }
    }

    // This guards against the case where the last used word contains no set bits in the used bits, but
    // one or more set bits in the unused bits after that.
    if (operator>=(end))
    {
        operator=(end);     // *this = end
        return (FALSE);
    }

    return (TRUE);
}


/**
While the iterator value is one, advance it.  If end is reached, return false.
\param end highest position to which the iterator can be advanced.
\return true if zero found, else false.
*/
inline RwpBool
BitArray::IteratorBase::FindZero(const IteratorBase &end)
{
    EA_ASSERT_MSG(m_bitIndex < BITS_PER_WORD, ("Iterator bit index not valid."));
    EA_ASSERT_MSG(!(end < *this), ("Iterator position is past end."));
    uint32_t bit = (uint32_t)(1 << m_bitIndex);

    while (*m_word & bit)
    {
        bit *= 2;
        if (++m_bitIndex == BITS_PER_WORD)
        {
            m_bitIndex = 0;
            bit = 1;
            do
            {
                if (++m_word > end.m_word)
                {
                    operator=(end);     // *this = end
                    return (FALSE);
                }
            }
            while (*m_word == ~WordType(0));
        }
    }
    return (operator<(end));      // *this < end
}


/**
Default constructor for an iterator.
*/
inline
BitArray::Iterator::Iterator()
  : BitArray::IteratorBase()
{
}

/**
Construct an iterator from a word pointer and a bit index within the word.
\param word a pointer to the a word in the array.
\param bitIndex the index of the bit within the word which we are interested in.
*/
inline
BitArray::Iterator::Iterator(BitArray::WordType *word, uint32_t bitIndex)
  : BitArray::IteratorBase(word, bitIndex)
{
}


/**
Construct an iterator from another iterator.
*/
inline
BitArray::Iterator::Iterator(const BitArray::Iterator &other)
  : BitArray::IteratorBase(other)
{
}


/**
Destructor.
*/
inline
BitArray::Iterator::~Iterator()
{
}

/**
Assign from another iterator.
*/
inline const BitArray::Iterator&
BitArray::Iterator::operator=(const BitArray::Iterator &other)
{
    BitArray::IteratorBase::operator=(other);
    return (*this);
}

/**
Move this iterator to the next bit.
*/
inline BitArray::Iterator&
BitArray::Iterator::operator++()
{
    MoveToNextBit();
    return (*this);
}

/**
Add distance to iterator.
\param distance number of bits to move.
\return an iterator position that is distance bits forward.
*/
inline BitArray::Iterator
BitArray::Iterator::operator+(uint32_t distance) const
{
    Iterator copy(*this);
    copy.MoveForward(distance);
    return (copy);
}

/**
Subtract distance from iterator.
\param distance number of bits to move.
\return an iterator position that is distance bits backward.
*/
inline BitArray::Iterator
BitArray::Iterator::operator-(uint32_t distance) const
{
    Iterator copy(*this);
    copy.MoveBackward(distance);
    return (copy);
}


/**
Set the bit to one.
*/
inline void
BitArray::Iterator::Set() const
{
    EA_ASSERT(m_bitIndex < BITS_PER_WORD);
    *m_word |= (1 << m_bitIndex);
}

/**
Clear the bit to zero.
*/
inline void
BitArray::Iterator::Unset() const
{
    EA_ASSERT(m_bitIndex < BITS_PER_WORD);
    *m_word &= ~(1 << m_bitIndex);
}



/**
Default construct an iterator.
*/
inline
BitArray::ConstIterator::ConstIterator()
  : BitArray::IteratorBase()
{
}

/**
Construct an iterator from a word pointer and a bit index within the word.
\param word a pointer to the a word in the array.
\param bitIndex the index of the bit within the word which we are interested in.
*/
inline
BitArray::ConstIterator::ConstIterator(BitArray::WordType *word, uint32_t bitIndex)
  : BitArray::IteratorBase(word, bitIndex)
{
}


/**
Construct an iterator from another iterator.
*/
inline
BitArray::ConstIterator::ConstIterator(const BitArray::IteratorBase &other)
  : BitArray::IteratorBase(other)
{
}


/**
Destructor.
*/
inline
BitArray::ConstIterator::~ConstIterator()
{
}

/**
Assign from another iterator.
*/
inline const BitArray::ConstIterator&
BitArray::ConstIterator::operator=(const BitArray::ConstIterator &other)
{
    BitArray::IteratorBase::operator=(other);
    return (*this);
}

/**
Move this iterator to the next bit.
*/
inline BitArray::ConstIterator&
BitArray::ConstIterator::operator++()
{
    MoveToNextBit();
    return (*this);
}

/**
Add distance to iterator.
\param distance number of bits to move.
\return an iterator position that is distance bits forward.
*/
inline BitArray::ConstIterator
BitArray::ConstIterator::operator+(uint32_t distance) const
{
    ConstIterator copy(*this);
    copy.MoveForward(distance);
    return (copy);
}

/**
Subtract distance from iterator.
\param distance number of bits to move.
\return an iterator position that is distance bits backward.
*/
inline BitArray::ConstIterator
BitArray::ConstIterator::operator-(uint32_t distance) const
{
    ConstIterator copy(*this);
    copy.MoveBackward(distance);
    return (copy);
}

inline uint32_t
BitArray::GetArraySize(uint32_t numBits)
{
    return sizeof(BitArray::WordType) * (EA::Physics::SizeAlign<uint32_t>(numBits, BITS_PER_WORD) >> WORD_SHIFT);
}

/**
Constructor
*/
inline
BitArray::BitArray()
  : m_data(0),
    m_sizeInBits(0),
    m_sizeInWords(0)
{
}


/**
Destructor
*/
inline
BitArray::~BitArray()
{
}



/**
Initialize the bit array
\param mem an array of 32-bit words to hold the bit data
\param sizeInBits the number of bits in the array
*/
inline void
BitArray::Initialize(BitArray::WordType *mem, uint32_t sizeInBits)
{
    m_data        = mem;
    m_sizeInBits  = sizeInBits;
    m_sizeInWords = ((m_sizeInBits + (BITS_PER_WORD - 1)) >> WORD_SHIFT);
}


/**
Get the memory size in bits
\return the memory size in bits
*/
inline uint32_t
BitArray::GetSize() const
{
    return (m_sizeInBits);
}


/**
Get the value of a bit.
\param index the index of the bit. For example index=0 gets the lowest bit of the first word in the array.
\return true if the bit is set, or false if it is clear
*/
inline RwpBool
BitArray::operator[](uint32_t index) const
{
    EA_ASSERT(index < m_sizeInBits);
    return ((m_data[index >> WORD_SHIFT] >> (index & (BITS_PER_WORD - 1))) & 1);
}

/**
Get the value of a bit. Throw an exception if the index is out of bounds
\param index the index of the bit. For example index=0 gets the lowest bit of the first word in the array.
\return true if the bit is set, or false if it is clear
*/
inline RwpBool
BitArray::at(uint32_t index) const
{
    EA_ASSERT_MSG(index < m_sizeInBits, ("BitArray index is greater than the size of the array."));
    return ((*this)[index]);
}



/**
Set a bit to one.
\param index index of the bit to set.
*/
inline void
BitArray::Set(uint32_t index)
{
    EA_ASSERT(index < m_sizeInBits);
    m_data[index >> WORD_SHIFT] |= (1 << (index & (BITS_PER_WORD - 1)));
}

/**
Clear a bit to zero.
\param index index of the bit to clear.
*/
inline void
BitArray::Unset(uint32_t index)
{
    EA_ASSERT(index < m_sizeInBits);
    m_data[index >> WORD_SHIFT] &= ~(1 << (index & (BITS_PER_WORD - 1)));
}

/**
Set all bits to one.
*/
inline void
BitArray::SetAll()
{
    WordType *pos = m_data, *last = m_data + m_sizeInWords;
    while (pos != last)
    {
        *pos++ = ~BitArray::WordType(0);
    }

}

/**
Clear all bits to zero.
*/
inline void
BitArray::UnsetAll()
{
    WordType *pos = m_data, *last = m_data + m_sizeInWords;
    while (pos != last)
    {
        *pos++ = BitArray::WordType(0);
    }

}

/**
Get a ConstIterator pointing to the start of the array.
*/
inline BitArray::ConstIterator
BitArray::Begin() const
{
    return (ConstIterator(m_data, 0));
}

/**
Get an Iterator pointing to the start of the array.
*/
inline BitArray::Iterator
BitArray::Begin()
{
    return (Iterator(m_data, 0));
}

/**
Get a ConstIterator pointing to the end of the array.
*/
inline BitArray::ConstIterator
BitArray::End() const
{
    return (ConstIterator(m_data + (m_sizeInBits >> WORD_SHIFT), m_sizeInBits & (BITS_PER_WORD - 1)));
}

/**
Get an Iterator pointing to the end of the array.
*/
inline BitArray::Iterator
BitArray::End()
{
    return (Iterator(m_data + (m_sizeInBits >> WORD_SHIFT), m_sizeInBits & (BITS_PER_WORD - 1)));
}

/**
Get a ConstIterator for a particular bit
\param index the bit index
\return an iterator referencing the indexed bit.
*/
inline BitArray::ConstIterator
BitArray::GetIterator(uint32_t index) const
{
    EA_ASSERT(index < m_sizeInBits);
    return (ConstIterator(m_data + (index >> WORD_SHIFT), index & (BITS_PER_WORD - 1)));
}

/**
Get an Iterator for a particular bit
\param index the bit index
\return an iterator referencing the indexed bit.
*/
inline BitArray::Iterator
BitArray::GetIterator(uint32_t index)
{
    EA_ASSERT(index < m_sizeInBits);
    return (Iterator(m_data + (index >> WORD_SHIFT), index & (BITS_PER_WORD - 1)));
}

/**
Get the index of the bit referenced by an iterator.
*/
inline uint32_t
BitArray::GetIndex(const IteratorBase &iter) const
{
    return (static_cast<uint32_t>((iter.m_word - m_data) * BITS_PER_WORD + iter.m_bitIndex));
}


/**
Get a word based iterator pointing to the start of the array.
*/
inline BitArray::ConstWordIterator
BitArray::WordBegin() const
{
    return (m_data);
}

/**
Get a word based iterator pointing to the start of the array.
*/
inline BitArray::WordIterator
BitArray::WordBegin()
{
    return (m_data);
}

/**
Get a word based iterator pointing to the end of the array.
*/
inline BitArray::ConstWordIterator
BitArray::WordEnd() const
{
    return (m_data + m_sizeInWords);
}

/**
Get a word based iterator pointing to the end of the array.
*/
inline BitArray::WordIterator
BitArray::WordEnd()
{
    return (m_data + m_sizeInWords);
}

#define BITARRAY_TO_MASK(value) ((uint32_t)(((int32_t)(value)) >> 31))
#define BITARRAY_SELECT(result, mask, a, b) (result = (a&mask) | (b&~mask))
#define BITARRAY_CONDITIONAL_ADD(result, mask, a, b) (result = a + (b & mask))

EA_FORCE_INLINE bool 
BitArray::AdvanceToOne_Branchless(WordType & word, uint32_t & index)
{

    uint32_t i=index;
    WordType w = word;

    uint32_t mask16 = ~BITARRAY_TO_MASK(0x0 - (w & 0xffff));
    BITARRAY_SELECT(w, mask16, w>>16, w);
    BITARRAY_CONDITIONAL_ADD(i, mask16, i, 16);

    uint32_t mask8 = ~BITARRAY_TO_MASK(0x0 - (w & 0xff));
    BITARRAY_SELECT(w, mask8, w>>8, w);
    BITARRAY_CONDITIONAL_ADD(i, mask8, i, 8);

    uint32_t mask4 = ~BITARRAY_TO_MASK(0x0 - (w & 0xf));
    BITARRAY_SELECT(w, mask4, w>>4, w);
    BITARRAY_CONDITIONAL_ADD(i, mask4, i, 4);

    uint32_t mask2 = ~BITARRAY_TO_MASK(0x0 - (w & 0x3));
    BITARRAY_SELECT(w, mask2, w>>2, w);
    BITARRAY_CONDITIONAL_ADD(i, mask2, i, 2);

    uint32_t mask1 = ~BITARRAY_TO_MASK(0x0 - (w & 0x1));
    BITARRAY_SELECT(w, mask1, w>>1, w);
    BITARRAY_CONDITIONAL_ADD(i, mask1, i, 1);

    index = i;
    word = w;

    return ((word & 0x1) == 1 && index < BITS_PER_WORD);
}

EA_FORCE_INLINE bool 
BitArray::AdvanceToZero_Branchless(WordType & word, uint32_t & index)
{
    uint32_t i=index;
    WordType w = word;

    uint32_t mask16 = ~BITARRAY_TO_MASK((w & 0xffff) - 0xffff);
    BITARRAY_SELECT(w, mask16, w>>16, w);
    BITARRAY_CONDITIONAL_ADD(i, mask16, i, 16);

    uint32_t mask8 = ~BITARRAY_TO_MASK((w & 0xff) - 0xff);
    BITARRAY_SELECT(w, mask8, w>>8, w);
    BITARRAY_CONDITIONAL_ADD(i, mask8, i, 8);

    uint32_t mask4 = ~BITARRAY_TO_MASK((w & 0xf) - 0xf);
    BITARRAY_SELECT(w, mask4, w>>4, w);
    BITARRAY_CONDITIONAL_ADD(i, mask4, i, 4);

    uint32_t mask2 = ~BITARRAY_TO_MASK((w & 0x3) - 0x3);
    BITARRAY_SELECT(w, mask2, w>>2, w);
    BITARRAY_CONDITIONAL_ADD(i, mask2, i, 2);

    uint32_t mask1 = ~BITARRAY_TO_MASK((w & 0x1) - 0x1);
    BITARRAY_SELECT(w, mask1, w>>1, w);
    BITARRAY_CONDITIONAL_ADD(i, mask1, i, 1);

    index = i;
    word = w;

    return ((w & 0x1) == 0 && i < BITS_PER_WORD);
}

EA_FORCE_INLINE bool 
BitArray::AdvanceToOne_Branching(WordType & word, uint32_t & index)
{
    uint32_t i=index;
    WordType w = word;
    if((w & 0xffff) == 0)
    {
        w >>= 16;
        i +=16;
    }

    if((w & 0xff) == 0)
    {
        w >>= 8;
        i +=8;
    }

    if((w & 0xf) == 0)
    {
        w >>= 4;
        i +=4;
    }

    if((w & 0x3) == 0)
    {
        w >>= 2;
        i +=2;
    }

    if((w & 0x1) == 0)
    {
        w >>= 1;
        i +=1;
    }

    index = i;
    word = w;

    return ((w & 0x1) == 1 && i < BITS_PER_WORD);
}

EA_FORCE_INLINE bool 
BitArray::AdvanceToZero_Branching(WordType & word, uint32_t & index)
{
    uint32_t i=index;
    WordType w = word;
    if((w & 0xffff) == 0xffff)
    {
        w >>= 16;
        i +=16;
    }

    if((w & 0xff) == 0xff)
    {
        w >>= 8;
        i +=8;
    }

    if((w & 0xf) == 0xf)
    {
        w >>= 4;
        i +=4;
    }

    if((w & 0x3) == 0x3)
    {
        w >>= 2;
        i +=2;
    }

    if((w & 0x1) == 0x1)
    {
        w >>= 1;
        i +=1;
    }

    index = i;
    word = w;

    return ((w & 0x1) == 0 && i < BITS_PER_WORD);
}

EA_FORCE_INLINE bool 
BitArray::AdvanceToOne(WordType & word, uint32_t & index)
{
    return AdvanceToOne_Branchless(word, index);
}

EA_FORCE_INLINE bool 
BitArray::AdvanceToZero(WordType & word, uint32_t & index)
{
    return AdvanceToZero_Branchless(word, index);
}

inline bool 
BitArray::FirstOne(uint32_t & index)
{
    WordType *wordPtr = m_data;
    WordType *end = m_data+m_sizeInWords;
    uint32_t wordIndex=0;
    while(*wordPtr == 0)
    {
        wordPtr++;
        wordIndex+=BITS_PER_WORD;
        if(wordPtr>end)
        {
            return false;
        }
    }

    WordType word = *wordPtr;

    uint32_t bitIndex=0;
    bool foundOne = AdvanceToOne(word, bitIndex);
    index = wordIndex + bitIndex;

    return foundOne && index < m_sizeInBits;
}

inline bool 
BitArray::FirstZero(uint32_t & index)
{
    WordType *wordPtr = m_data;
    WordType *end = m_data+m_sizeInWords;
    uint32_t wordIndex=0;

    while(wordPtr[0] == 0xffffffff && 
          wordPtr[1] == 0xffffffff && 
          wordPtr[2] == 0xffffffff && 
          wordPtr[3] == 0xffffffff && &wordPtr[3] <end)
    {
        wordPtr+=4;
        wordIndex+=BITS_PER_WORD * 4;
    }

    while(*wordPtr == 0xffffffff && wordPtr<end)
    {
        wordPtr++;
        wordIndex+=BITS_PER_WORD;
    }

    if(wordPtr>end)
    {
        return false;
    }

    WordType word = *wordPtr;

    uint32_t bitIndex=0;
    bool foundZero = AdvanceToZero(word, bitIndex);
    index = wordIndex + bitIndex;

    return foundZero && index < m_sizeInBits;
}

template <class T>
inline void 
BitArray::IterateOnes(T & handler, uint32_t startWord, uint32_t wordIncrement)
{
    WordType *wordPtr = &m_data[startWord];
    WordType *end = m_data+m_sizeInWords;
    uint32_t wordIndex=startWord;
    while(wordPtr < end)
    {
        if(*wordPtr != 0)
        {
            uint32_t word = *wordPtr;
            uint32_t bitIndex=0;
            uint32_t arrayIndex = (wordIndex * BITS_PER_WORD);
            while((word &0x1)==0x1 || AdvanceToOne(word, bitIndex))
            {
                arrayIndex = (wordIndex * BITS_PER_WORD) + bitIndex;
                if(arrayIndex >= m_sizeInBits)
                {
                    break;
                }
                handler.Process(arrayIndex);
                word >>= 1;
                ++bitIndex;
            }
        }

        wordPtr+=wordIncrement;
        wordIndex+=wordIncrement;
    }
}

template <class T>
inline void 
BitArray::IterateZeros(T & handler, uint32_t startWord, uint32_t wordIncrement)
{
    WordType *wordPtr = &m_data[startWord];
    WordType *end = m_data+m_sizeInWords;
    uint32_t wordIndex=startWord;
    while(wordPtr < end)
    {
        if((~*wordPtr) != 0)
        {
            uint32_t word = *wordPtr;
            word = ~word;
            uint32_t bitIndex=0;
            uint32_t arrayIndex = (wordIndex * BITS_PER_WORD);
            while((word &0x1)==0x1 || AdvanceToOne(word, bitIndex))
            {
                arrayIndex = (wordIndex * BITS_PER_WORD) + bitIndex;
                if(arrayIndex >= m_sizeInBits)
                {
                    break;
                }

                handler.Process(arrayIndex);
                word >>= 1;
                ++bitIndex;
            }
        }

        wordPtr+=wordIncrement;
        wordIndex+=wordIncrement;
    }
}

inline void 
BitArray::CopyFrom(const BitArray & other)
{
    EA_ASSERT_MSG(other.m_data != m_data, ("Source and destimation identical"));
    static const uint32_t BYTES_PER_WORD = 4;
    uint32_t size = other.m_sizeInWords;
    EA_ASSERT_MSG(size <= m_sizeInWords, ("size of other larger than sizeof this"));
    memcpy(m_data, other.m_data, size*BYTES_PER_WORD);
}

inline void 
BitArray::Not()
{
    for(uint32_t i=0; i<m_sizeInWords; i++)
    {
        m_data[i] = ~m_data[i];
    }
}

inline void 
BitArray::And(const BitArray & rhs)
{
    EA_ASSERT_MSG(rhs.m_sizeInWords <= this->m_sizeInWords, ("source array too big"));
    for(uint32_t i=0; i<rhs.m_sizeInWords; i++)
    {
        m_data[i] = m_data[i] & rhs.m_data[i];
    }
}

inline void 
BitArray::Or(const BitArray & rhs)
{
    EA_ASSERT_MSG(rhs.m_sizeInWords <= this->m_sizeInWords, ("source array too big"));
    for(uint32_t i=0; i<rhs.m_sizeInWords; i++)
    {
        m_data[i] = m_data[i] | rhs.m_data[i];
    }
}

}} //namespace rw::collision

#endif // RW_COLLISION_BITARRAY_H
