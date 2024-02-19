class Tuple implements Comparable<Tuple>{
    float distance;
    int idx;

    Tuple(float distance, int idx){
        this.distance = distance;
        this.idx = idx;
    }

    @Override
    int compareTo(Tuple t){
        if(this.distance < t.distance){
            return -1;
        } else if (this.distance > t.distance) {
            return 1;
        } else {
            return 0;
        }
    }
}