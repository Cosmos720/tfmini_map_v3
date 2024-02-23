//  Concave Hull k-nearest neighbors OG Algorithm
ArrayList<Points> computeConcaveHull(ArrayList<Points> pointsList, int k) {
  k = max(k, 3);
  ArrayList<Points> dataset = cleanList(pointsList);
  if(dataset.size()<3){
    return null;
  }
  if(dataset.size()==3){
    return dataset;
  }
  k = min(k, dataset.size()-1);
  Points firstPoint = findMinYPoint(dataset);
  ArrayList<Points> hull = new ArrayList<Points>();
  hull.add(firstPoint);
  Points currentPoint = firstPoint;
  dataset.remove(firstPoint);
  float previousAngle = 0;
  int step = 1;
  while (((currentPoint!=firstPoint) || (step==1)) && dataset.size()>0) {
    if(step==4){
      dataset.add(firstPoint);
    }
    ArrayList<Points> kNearestPoints = nearestPoints(dataset, currentPoint, k);
    ArrayList<Points> cPoints = sortByAngle(kNearestPoints, currentPoint, previousAngle);
    boolean its = true;
    int i = -1;
    while ((its==true)&(i<cPoints.size()-1)) {
      i++;
      int lastPoint = 0;
      if(cPoints.get(i).equals(firstPoint)){
        lastPoint = 1;
      }
      int j = 1;
      its = false;
      while ((its==false)&(j<hull.size()-lastPoint-1)) {
        its = interstectsQ(hull.get(step-1), cPoints.get(i), hull.get(step-j-1), hull.get(step-j));
        j++;
      }
    }
    if(its==true & k<dataset.size()){
      println(k + " Que des intersections \n"+hull);
      //return hull;
      return computeConcaveHull(pointsList, k+1);
    } else if (its==true) {
      return hull;
    }
    currentPoint = cPoints.get(i);
    hull.add(currentPoint);
    previousAngle = angleCW(hull.get(step-1), hull.get(step));//+(180-previousAngle);
    dataset.remove(currentPoint);
    step++;
  }
  boolean allInside = true;
  int i = dataset.size()-1;
  while(allInside & i>0){
    allInside = pointInPolygonQ(dataset.get(i), hull);
    i--;
  }
  if(!allInside & k<dataset.size()){
    println(k + " not all inside \n"+hull);
    return computeConcaveHull(pointsList, k+1);
    //return hull;
  }
  println(k);
  return hull;
}

boolean pointInPolygonQ(Points p, ArrayList<Points> hull){
  Points ray = new Points(p.xCoord, findMinYPoint(hull).yCoord-1);
  boolean interieur = false;
  for(int i = 0; i<hull.size(); i++){
    if(interstectsQ(ray, p, hull.get(i), hull.get((i+1)%hull.size()))){
      interieur = !interieur;
    }
  }
  return interieur;
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

/*
ArrayList<Points> sortByAngle(ArrayList<Points> data, Points pivot, float prevAngle){
  ArrayList<Points> tmp = (ArrayList<Points>)data.clone();
  tmp.sort((p1, p2) -> ((angleCW(pivot, p1)+(180-prevAngle))-(angleCW(pivot, p2)+(180-prevAngle))<0?1:-1));
  return tmp;
}

// angle Op1p2
float angleCW(Points pivot, Points destination){
  float angle = degrees(atan2((destination.yCoord - pivot.yCoord), (destination.xCoord - pivot.xCoord)));
  return angle-360<=360?angle-360:-angle;
}
*/

// GH = on ajoute 360 et on calcule % 360 pour se ramener dans [0..360]
ArrayList<Points> sortByAngle(ArrayList<Points> data, Points pivot, float prevAngle){
  ArrayList<Points> tmp = (ArrayList<Points>)data.clone();
  tmp.sort((p1, p2) -> (((angleCW(pivot, p1)-prevAngle+540)%360) < ((angleCW(pivot, p2)-prevAngle+540)%360) ? 1 : -1));
  return tmp;
}

// GH = on renvoie simplement le résultat de atan2
// angle Op1p2
float angleCW(Points pivot, Points destination){
  float angle = degrees(atan2((destination.yCoord - pivot.yCoord), (destination.xCoord - pivot.xCoord)));
  return angle;
}

ArrayList<Points> nearestPoints(ArrayList<Points> data, Points point, int neighbors){
  ArrayList<Tuple> dist = new ArrayList<Tuple>();
  ArrayList<Points> res = new ArrayList<Points>();
  for(int i = 0; i<data.size(); i++){
    dist.add(new Tuple(point.distance(data.get(i)), i));
  }
  // GH : j'ai enlevé le -1 à dist.size()
  int kk = min(neighbors, dist.size());
  dist.sort((d1, d2) -> d1.compareTo(d2));
  // GH : j'ai décalé les indices = de 0 à kk au lieu de 1 à kk+1 
  dist.subList(0, kk).forEach((t) -> res.add(data.get(t.idx)));
  return res;
}

Points findMinYPoint(ArrayList<Points> list){
  ArrayList<Points> tmp = (ArrayList<Points>)list.clone();
  tmp.sort((p1, p2) -> p1.compareTo(p2));
  return tmp.get(0);
}
/*
boolean contains(ArrayList<Points> list, Points p) {
  for (int i=1; i<list.size(); i++) {
    if ((list.get(i).xCoord == p.xCoord) &&
      (list.get(i).yCoord == p.yCoord))
      return true;
    }
    return false;
}
*/

ArrayList<Points> cleanList(ArrayList<Points> list){
  ArrayList<Points> pointsSet = new ArrayList<Points>();
  pointsSet.add(list.get(0));
  for(int i=1; i<list.size(); i++){
    if(!pointsSet.contains(list.get(i))){
      pointsSet.add(list.get(i));
    }
    /*if(!contains(pointsSet, list.get(i))){
      pointsSet.add(list.get(i));
    }*/
  }
  return pointsSet;
}
