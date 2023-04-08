/**
 * Quite simple ring buffer, copied from https://www.approxion.com/circular-adventures-vii-a-ring-buffer-implementation/
 *
 * Adapted to pointer usage so that buffer doesn't get created in heap
 */

template <typename T, T *P, int N>
class ring_buffer
{
public:
    ring_buffer() { clear(); }
    // Unused: size_t capacity() const { return N; }
    bool empty() const { return head_ == tail_; }
    size_t size() const
    {
        return head_ >= tail_ ? head_ - tail_ : BUFSIZE - (tail_ - head_);
    }
    void add(const T &item)
    {
        *(P + head_) = item;
        advance(head_);
        if (head_ == tail_)
        {
            advance(tail_); // Drop oldest entry, keep rest.
        }
    }
    const T &remove()
    {
        // FIXME: assert(!empty());
        size_t old_tail = tail_;
        advance(tail_);
        return *(P + old_tail);
    }
    void clear() { tail_ = head_ = 0U; }

private:
    static const size_t BUFSIZE = N - 1U;
    void advance(size_t &value) { value = (value + 1) % BUFSIZE; }

    size_t head_;
    size_t tail_;
};
