//  Concave Hull k-nearest neighbors
ArrayList<Points> computeConcaveHull(ArrayList<Points> pointsList, int k){
  k = max(k,3);
  ArrayList<Points> dataset = cleanList(pointsList);
  if(dataset.size()<3){
    return null;
  }
  if(dataset.size()=3){
    return dataset;
  }
  k = min(k, dataset.size()-1);
  Points firstPoint = findMinYPoint(dataset);
  ArrayList<Points> hull = new ArrayList<Points>();
  hull.add(firstPoint);
  Points currentPoint = firstPoint;
  dataset.remove(firstPoint);
  float previousAngle = 0;
  int step = 2;
  while (((currentPoint!=firstPoint) || (step==2)) && dataset.size()>0) {
    if(step==5){
      dataset.add(firstPoint);
    }
    ArrayList<Points> kNearestPoints = nearestPoints(dataset, currentPoint, k);
    ArrayList<Points> cPoints = sortByAngle(kNearestPoints, currentPoint, previousAngle);
    boolean its = true;
    int i = 0;
    while ((its==true)&(i<cPoints.size())) {
      i++;
      if(cPoints.get(i).equals(firstPoint)){
        int lastPoint = 1;
      } else {
        int lastPoint = 0;
      }
      int j = 2;
      its = false;
      while ((its==false)&(j<hull.size()-lastPoint)) {
        its = interstectsQ(hull.get(step-1), cPoints.get(i), hull(step-1-j), hull.get(step-j));
        j++;
      }
      if(its==true){
        return computeConcaveHull(pointsList, k+1);
      }
      currentPoint = cPoints.get(i);
      hull.add(currentPoint);
      prevAngle = angleCW(hull.get(step-1), hull.get(step))+(180-prevAngle);
      dataset.remove(currentPoint);
      step++;
    }
    boolean allInside = true;
    i = dataset.size();
    while(allInside & i>0){
      allInside = pointInPolygonQ(dataset.get(i), hull);
      i--;
    }
    if(!allInside){
      return computeConcaveHull(pointsList, k+1);
    }
  }
  return hull;
}

boolean pointInPolygonQ(Points p, ArrayList<Points> hull){
  
}

boolean interstectsQ(Points a, Points b, Points c, Points d){
  float[] AB = a.vectorize(b);
  float[] AC = a.vectorize(c);
  float[] AD = a.vectorize(d);
  float[] CD = c.vectorize(d);
  float[] CA = c.vectorize(a);
  float[] CB = c.vectorize(b);
  boolean partAutreCD = prodVec(AB, AC)*prodVec(AB, AD)<0;
  boolean partAutreAB = prodVec(CD, CA)*prodVec(CD, CB)<0;
  if (partAutreAB & partAutreCD){
    return true;
  }else{
    return false;
  }
}

float prodVec(float[] v1, float[] v2){
  return v1[0]*v2[1] - v1[1]*v2[0];
}

ArrayList<Points> sortByAngle(ArrayList<Points> data, Points pivot, float prevAngle){
  ArrayList<Points> tmp = (ArrayList<Points>)data.clone();
  tmp.sort((p1, p2) -> ((angleCW(pivot, p1)+(180-prevAngle))-(angleCW(pivot, p2)+(180-prevAngle))<0?1:-1));
  return tmp;
}

// angle Op1p2
float angleCW(Points pivot, Points destination){
  float angle = degrees(atan2((destination.yCoord - pivot.yCoord),(destination.xCoord - pivot.xCoord)));
  return angle-360<=360?angle-360:-angle;
}

ArrayList<Points> nearestPoints(ArrayList<Points> data, Points point, int neighbors){
  ArrayList<Tuple> dist = new ArrayList<Tuple>();
  ArrayList<Points> res = new ArrayList<Points>();
  for(int i = 0; i<data.size(); i++){
    dist.add(new Tuple(point.distance(data.get(i)), i));
  }
  dist.sort((d1, d2) -> d1.compareTo(d2));
  dist.subList(1,neighbors+1).forEach((t) -> res.add(data.get(t.idx)));
  return res;
}

Points findMinYPoint(ArrayList<Points> list){
  ArrayList<Points> tmp = (ArrayList<Points>)list.clone();
  tmp.sort((p1, p2) -> p1.compareTo(p2));
  return tmp.get(0);
}

ArrayList<Points> cleanList(ArrayList<Points> list){
  ArrayList<Points> pointsSet = new ArrayList<Points>();
  pointsSet.add(list.get(0));
  for(int i=1; i<list.size(); i++){
    if(!pointsSet.contains(list.get(i))){
      pointsSet.add(list.get(i));
    }
  }
  return pointsSet;
}
