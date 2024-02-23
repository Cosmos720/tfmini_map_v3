class Tuple implements Comparable<Tuple>{
    float distance;
    int idx;

    Tuple(float distance, int idx){
        this.distance = distance;
        this.idx = idx;
    }

    @Override
    int compareTo(Tuple t){
        return (this.distance - t.distance)<0?-1:1;
    }
}