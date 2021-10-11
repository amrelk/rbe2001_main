template <class T, int N>
class RollAvg
{
private:
    T reads [N];
    int readIndex = 0;
    T total = 0;
public:
    RollAvg();
    void addRead(T reading);
    T getAvg();
};

template <class T, int N>
RollAvg<T,N>::RollAvg() {
    for (int i = 0; i < N; i++) {
        RollAvg::reads[i] = 0;
    }
}

template <class T, int N>
void RollAvg<T,N>::addRead(T reading) {
    total -= reads[readIndex];
    reads[readIndex] = reading;
    total += reads[readIndex];
    readIndex++;
    if (readIndex >= N) {
        readIndex = 0;
    }
}

template<class T, int N>
T RollAvg<T,N>::getAvg() {
    return total/N;
}