//  *Concave Hull k-nearest neighbors OG Algorithm
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
        its = intersectsQ(hull.get(step-1), cPoints.get(i), hull.get(step-j-1), hull.get(step-j));
        j++;
      }
    }
    if(its==true & k<dataset.size()){
      println(k + " Que des intersections \n"+hull);
      return computeConcaveHull(pointsList, k+1);
    }
    currentPoint = cPoints.get(i);
    hull.add(currentPoint);
    previousAngle = angleCW(hull.get(step-1), hull.get(step));
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
  }
  println(k);
  return hull;
}

// *iteratively
//! Je suis BLOQUÉ
ArrayList<Points> computeConcaveHullNew(ArrayList<Points> pointsList, int k){
  int nb=0;
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
  boolean valid = false;
  int kMAX = k;
  int kNeigh = k;
  ArrayList<Points> tmp = new ArrayList<Points>();
  while(!valid){
    while((nb != nbIter) && ((currentPoint!=firstPoint) || (step==1)) && dataset.size()>0) {
      nb++;
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
          its = intersectsQ(hull.get(step-1), cPoints.get(i), hull.get(step-j-1), hull.get(step-j));
          j++;
        }
      }
      if(its==true){
        //println(step + " Que des intersections => iteration\n"+hull);
        if(k>=kMAX){
          step--;
          tmp.add(currentPoint);
          currentPoint = hull.get(step);
          hull.remove(step);
        } else {
          k++;
        }
        continue;
      }else{
        k = kNeigh;
      }
      currentPoint = cPoints.get(i);
      hull.add(currentPoint);
      previousAngle = angleCW(hull.get(step-1), hull.get(step));
      dataset.remove(currentPoint);
      step++;
    }
    boolean allInside = true;
    int i = dataset.size()-1;
    while(allInside & i>0){
      allInside = pointInPolygonQ(dataset.get(i), hull);
      i--;
    }
    if(!allInside & nb < nbIter){
      //println(step + " not all inside => recompute\n"+hull);
      step--;
      currentPoint = hull.get(step);
      //println(step + " " + currentPoint);
      hull.remove(step);
      if(currentPoint == firstPoint){
        //println("remove et add first");
        dataset.add(firstPoint);
      }
      kMAX++;
      dataset.addAll(tmp.isEmpty()?tmp:cleanList(tmp));
      tmp.clear();
      continue;
    }else{
      println("allInside:" + allInside);
      valid = true;
    }
  }
  println(k + " " + kMAX);
  return hull;
}

ArrayList<Edge> computeConcaveHullFromConvexe(ArrayList<Points> pointsList, float n){
  ArrayList<Points> points = (ArrayList<Points>)pointsList.clone();
  int[] hull = computeEnveloppe();
  int nb=0;
  ArrayList<Edge> convexeHull = new ArrayList<Edge>();
  for(int i = 0; i<hull.length; i++){
    convexeHull.add(new Edge(points.get(hull[i]), points.get(hull[(i+1)%hull.length])));
  }
  for(int i = hull.length-1; i>=0; i--){
    points.remove(hull[i]);
  }
  ArrayList<Edge> concaveHull = (ArrayList<Edge>)convexeHull.clone();
  for(int i=0; i<concaveHull.size(); i++){
    if (nb<nbIter){
      nb++;
    }else{
      break;
    }
    println(concaveHull.size());
    Edge[] neighbors = neighborsEdge(concaveHull.get(i), concaveHull);
    Points pk = nearestInnerPoint(concaveHull.get(i), neighbors, points);
    if (pk==null){
      continue;
    }
    float eh = concaveHull.get(i).distance();
    float dd = decisionDistance(pk, concaveHull.get(i));
    if ((eh/dd)>n){
      Edge e = concaveHull.remove(i);
      concaveHull.add(new Edge(pk, e.point1));
      concaveHull.add(new Edge(pk, e.point2));
      points.remove(pk);

    }
  }
  println(concaveHull);
  return concaveHull;
}

float decisionDistance(Points p, Edge e){
  return min(p.distance(e.point1), p.distance(e.point2));
}

Points nearestInnerPoint(Edge e, Edge[] neighbors, ArrayList<Points> pointsList){
  ArrayList<Points> tmp = (ArrayList<Points>)pointsList.clone();
  tmp.sort((p1, p2) -> (distEdge(p1, e) < distEdge(p2, e))?-1:1);
  for(int i=0; i<tmp.size(); i++){
    if(distEdge(tmp.get(i), neighbors[0])>distEdge(tmp.get(i), e) && distEdge(tmp.get(i), neighbors[1])>distEdge(tmp.get(i), e)){
      return tmp.get(i);
    }
  }
  return null;
}

Edge[] neighborsEdge(Edge e, ArrayList<Edge> edges){
  Edge[] res = new Edge[2];
  for(int i=0; i<edges.size(); i++){
    if(edges.get(i).equals(e)){
      continue;
    }
    if(edges.get(i).point1.equals(e.point1) || edges.get(i).point2.equals(e.point1)){
      res[0] = edges.get(i);
    }
    if(edges.get(i).point1.equals(e.point2) || edges.get(i).point2.equals(e.point2)){
      res[1] = edges.get(i);
    }
  }
  return res;
}

float distEdge(Points p, Edge e){
  return min(p.distance(e.point1), p.distance(e.point2));
}

boolean pointInPolygonQ(Points p, ArrayList<Points> hull){
  Points ray = new Points(p.xCoord, findMinYPoint(hull).yCoord-1);
  boolean interieur = false;
  for(int i = 0; i<hull.size(); i++){
    if(intersectsQ(ray, p, hull.get(i), hull.get((i+1)%hull.size()))){
      interieur = !interieur;
    }
  }
  return interieur;
}

boolean intersectsQ(Points a, Points b, Points c, Points d){
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
