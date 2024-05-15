class Points implements Comparable<Points>{
    float xCoord, yCoord;
    float angle;
    
    Points(float x, float y, float a ){
        xCoord = x;
        yCoord = y;
        angle=a;
    }

    Points(float x, float y){
        xCoord = x;
        yCoord = y;
        angle=-1;
    }

    @Override
    boolean equals(Object o){
        if (o == this){
            return true;
        }
        if (!(o instanceof Points)){
            return false;
        }
        Points p = (Points)o;
        return (this.xCoord==p.xCoord & this.yCoord==p.yCoord);
    }

    @Override
    int compareTo(Points p){
        if(this.yCoord<p.yCoord || (this.yCoord == p.yCoord && this.xCoord<p.xCoord)){
            return -1;
        } else if (this.yCoord>p.yCoord || (this.yCoord == p.yCoord && this.xCoord>p.xCoord)) {
            return 1;
        } else {
            return 0;
        }
    }

    //Euclidean distance
    float distance(Points p){
        return sqrt(pow(this.xCoord - p.xCoord, 2) + pow(this.yCoord - p.yCoord, 2));
    }

    float[] vectorize(Points p){
        float[] tmp = {p.xCoord-this.xCoord, p.yCoord-this.yCoord};
        return tmp;
    }
    
    @Override
    String toString(){
      return String.format("(%.1f, %.1f): %.1f",xCoord, yCoord, angle);
    }
}