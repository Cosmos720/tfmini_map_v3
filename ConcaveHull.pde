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
      while ((its==false)&(j<hull.size()-lastPoint)) {
        its = intersectsQ(hull.get(step-1), cPoints.get(i), hull.get(step-j-1), hull.get(step-j));
        j++;
      }
    }
    if(its==true){
      //println(k + " Que des intersections \n"+hull);
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
  if(!allInside){
    //println(k + " not all inside \n"+hull);
    return computeConcaveHull(pointsList, k+1);
  }
  //println(k);
  return hull;
}

// *iteratively
/*
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
*/
ArrayList<Edge> computeConcaveHullFromConvexe(ArrayList<Points> pointsList, float n){
  ArrayList<Points> points = (ArrayList<Points>)pointsList.clone();
  int[] hull = computeEnveloppe();
  //int nb=1;
  ArrayList<Edge> convexeHull = new ArrayList<Edge>();
  for(int i = 0; i<hull.length; i++){
    convexeHull.add(new Edge(points.get(hull[i]), points.get(hull[(i+1)%hull.length])));
  }
  for(int i = hull.length-1; i>=0; i--){
    points.remove(hull[i]);
  }

  /*
  stroke(16, 100, 100, 10);
  for(int i=0; i<convexeHull.size(); i++){
    line(convexeHull.get(i).point1.xCoord, convexeHull.get(i).point1.yCoord, convexeHull.get(i).point2.xCoord, convexeHull.get(i).point2.yCoord);
  }
  */
  
  ArrayList<Edge> concaveHull = (ArrayList<Edge>)convexeHull.clone();
  IntList del = new IntList();
  for(int i=0; i<concaveHull.size(); i++){
    Edge[] neighbors = neighborsEdge(concaveHull.get(i), concaveHull, del);
    Points pk = nearestInnerPoint(concaveHull.get(i), neighbors, points, concaveHull);
    if (pk==null){
      continue;
    }
   /* if (nb<nbIter){
      nb++;
    }else{
      stroke(0, 100, 50);
      strokeWeight(20);
      line(concaveHull.get(i).point1.xCoord-xmin, concaveHull.get(i).point1.yCoord-ymin, concaveHull.get(i).point2.xCoord-xmin, concaveHull.get(i).point2.yCoord-ymin);
      stroke(180, 100, 50);
      line(neighbors[0].point1.xCoord-xmin,neighbors[0].point1.yCoord-ymin,neighbors[0].point2.xCoord-xmin,neighbors[0].point2.yCoord-ymin);
      line(neighbors[1].point1.xCoord-xmin,neighbors[1].point1.yCoord-ymin,neighbors[1].point2.xCoord-xmin,neighbors[1].point2.yCoord-ymin);
      stroke(270, 100, 100);
      strokeWeight(30);
      point(pk.xCoord-xmin, pk.yCoord-ymin);
      break;
    }*/
    float eh = concaveHull.get(i).distance();
    float dd = distEdge(pk, concaveHull.get(i));//decisionDistance(pk, concaveHull.get(i));
    if ((eh/dd)>n){
      del.append(i);
      concaveHull.add(new Edge(concaveHull.get(i).point1, pk));
      concaveHull.add(new Edge(pk, concaveHull.get(i).point2));
      points.remove(pk);
    }
  }
  for(int i = del.size()-1; i>=0; i--){
    concaveHull.remove(del.get(i));
  }

  concaveHull = sortEdgeHull(concaveHull, pointsList.get(0));

  for (int i = 0; i < concaveHull.size(); ++i) {
    Edge edge = concaveHull.get(i);
    if(abs(edge.point2.angle-edge.point1.angle) != 1){
      Points inter = intersectionPoint(concaveHull.get((i-1)<0?concaveHull.size()-1:i-1), concaveHull.get((i+1)%concaveHull.size()));
      if(inter==null){
        continue;
      }
      float[] e1 = edge.point1.vectorize(inter);
      float[] e2 = inter.vectorize(edge.point2);
      float prodScal = e1[0]*e2[0]+e1[1]*e2[1];
      float e1Norm = sqrt(sq(e1[0])+sq(e1[1]));
      float e2Norm = sqrt(sq(e2[0])+sq(e2[1]));
      float angle = degrees(acos(prodScal/(e1Norm*e2Norm)));
      if(angle<45 || angle>135){
        continue;
      }
      ArrayList<Points> concaveHullPts = new ArrayList<Points>();
      for (Edge e : concaveHull) {
        concaveHullPts.add(e.point1);
      }
      if(!pointInPolygonQ(inter, concaveHullPts)){
        concaveHull.remove(edge);
        concaveHull.add(i, new Edge(edge.point1, inter));
        concaveHull.add(i+1, new Edge(inter, edge.point2));
        i++;
      }
    }
  }
  
  return concaveHull;
}

float decisionDistance(Points p, Edge e){
  return min(p.distance(e.point1), p.distance(e.point2));
}

Points nearestInnerPoint(Edge e, Edge[] neighbors, ArrayList<Points> pointsList, ArrayList<Edge> hull){
  ArrayList<Points> tmp = (ArrayList<Points>)pointsList.clone();
  tmp.sort((p1, p2) -> (distEdge(p1, e) < distEdge(p2, e))?-1:1);
  findPoint:
  for(int i=0; i<tmp.size(); i++){
    if(distEdge(tmp.get(i), neighbors[0])<distEdge(tmp.get(i), e) || distEdge(tmp.get(i), neighbors[1])<distEdge(tmp.get(i), e)){
      continue;
    }
    for(int j=0; j<hull.size(); j++){
      Edge edges = hull.get(j);
      if(intersectsQ(e.point1, tmp.get(i), edges.point1, edges.point2) || intersectsQ(tmp.get(i), e.point2, edges.point1, edges.point2)){
        continue findPoint;
      }
    }
    return tmp.get(i);
  }
  return null;
}

Edge[] neighborsEdge(Edge e, ArrayList<Edge> edges, IntList del){
  Edge[] res = new Edge[2];
  for(int i=0; i<edges.size(); i++){
    if(edges.get(i).equals(e) || del.hasValue(i)){
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


/*
            (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
        s = -----------------------------
                        L^2
*/
float distEdge(Points p, Edge e){
  ////return min(p.distance(e.point1), p.distance(e.point2));
  float s = ((e.point1.yCoord-p.yCoord)*(e.point2.xCoord - e.point1.xCoord)-(e.point1.xCoord-p.xCoord)*(e.point2.yCoord - e.point1.yCoord))/pow(e.distance(), 2);
  float r = ((p.xCoord-e.point1.xCoord)*(e.point2.xCoord - e.point1.xCoord)+(p.yCoord-e.point1.yCoord)*(e.point2.yCoord - e.point1.yCoord))/pow(e.distance(), 2);
  ////return abs(s)*e.distance();
  if(r<=0){
    return p.distance(e.point1);
  }else if(r>=1){
    return p.distance(e.point2);
  }else{
    return abs(s)*e.distance();
  }
}

Points intersectionPoint(Edge ab, Edge cd){
  Points a = ab.point1;
  Points b = ab.point2;
  Points c = cd.point1;
  Points d = cd.point2;
  float[] AB = a.vectorize(b);
  float[] CD = c.vectorize(d);

  float[] AC = a.vectorize(c);

  float divisor = AB[0] * CD[1] - AB[1] * CD[0];

  if (divisor == 0){
    // Les droites sont parallèles ou coïncident
    return null;
  }

  float t = (AC[0] * CD[1] - AC[1] * CD[0]) / divisor;
  float u = (AC[0] * AB[1] - AC[1] * AB[0]) / divisor;

  if ((t < 0 || t > 1) || (u < 0 || u > 1)){
    float[] intersection = {a.xCoord + t * AB[0], a.yCoord + t * AB[1]};
    return new Points(intersection[0], intersection[1], (b.angle+c.angle)/2);
  }else{
    return null;
  }
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
