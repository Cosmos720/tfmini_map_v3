/*
BufferedReader log;
String line = null;

int[][] mesures = new int[121][3];
int[] strengths = new int[363];
float[] angles = new float[363];
int[] enveloppe = new int[363];
int pile = 1;

String[] pieces, pieces2;
int i, j, d, s;
float alpha, gamma, l, r;
float points[][];

float xmin, xmax, ymin, ymax;
int xmin_id, xmax_id, ymin_id, ymax_id;

void swapPoints(int i, int j) {
  float tmpx = points[i][0];
  float tmpy = points[i][1];
  float tmpa = angles[i];
  points[i][0] = points[j][0];
  points[i][1] = points[j][1];
  points[j][0] = tmpx;
  points[j][1] = tmpy;
  angles[j] = tmpa;
}

//j<i
void insertPointsAngles(int i, int j) { 
  float tmpx = points[i][0];
  float tmpy = points[i][1];
  float tmpa = angles[i];

  for (int k=i; k>j; k--) {
    points[k][0] = points[k-1][0];
    points[k][1] = points[k-1][1];
    angles[k] = angles[k-1];
  }
  points[j][0] = tmpx;
  points[j][1] = tmpy;
  angles[j] = tmpa;
}

void minXY() {
  xmin = xmax = points[0][0];
  xmin_id = 0;
  xmax_id = 0;
  ymin = ymax = points[0][1];
  ymin_id = 0;
  ymax_id = 0;

  for (int i=1; i<363; i++) {
    //  println("(" + points[i][0] + "," + points[i][1] + ")");
    if (points[i][0] < xmin) {
      xmin = points[i][0];
      xmin_id = i;
    } else if (points[i][0] > xmax) {
      xmax = points[i][0];
      xmax_id = i;
    }
    if (points[i][1] < ymin) {
      ymin = points[i][1];
      ymin_id = i;
    } else if (points[i][1] > ymax) {
      ymax = points[i][1];
      ymax_id = i;
    }
  }

  println("xmin = " + xmin);
  println("xmax = " + xmax);
  println("ymin = " + ymin);
  println("ymax = " + ymax);
}

void sortPoints() {
  swapPoints(0, ymin_id);
  int j;

  for (int i=1; i<363; i++) {
    angles[i] = (points[i][0] - points[0][0]) /
      (points[i][1] - points[0][1]);

    j = 1;
    while ((j < i) && (angles[j] < angles[i]))
      j++;
    if (j != i)
      insertPointsAngles(i, j);
  }
  println(angles);
}

float prodVec(int a, int b, int c) {
  return (points[b][0] - points[a][0]) * (points[c][1] - points[a][1]) -
  (points[c][0] - points[a][0]) * (points[b][1] - points[a][1]);
}

// Graham scan
void computeEnveloppe() {
  enveloppe[0] = 0;
  enveloppe[1] = 1;
  for (int i = 3; i<363; i++) {
    while ((pile >=1) && prodVec(enveloppe[pile-1], enveloppe[pile], i) > 0)
      pile--;
    pile++;
    enveloppe[pile] = i;
  }
  for (int i=pile+1; i<363; i++)
    enveloppe[i] = 0;
  println(enveloppe);
}

void setup() {
  size(1000, 1000);
  scale(0.25);
  log = createReader("../LaLigneRouge_v2_0/output/LIDAR.log");
  //log = createReader("data/tfmini14.log");
  try {
    while ((line = log.readLine()) != null) {
      pieces = split(line, "->");
      pieces = split(pieces[1], " ");
      pieces2 = split(pieces[1], ",");
      i = int(pieces2[0]);
      j = int(pieces2[1]);
      d = int(pieces2[2]);
      s = int(pieces2[3]);
      mesures[i][j] = d;
      strengths[j*121+i] = s;
    }
    log.close();
  }
  catch (IOException e) {
    e.printStackTrace();
  }

  points = new float[363][2];
  r = 18;

  float smin, smax;

  smin = smax = strengths[0];

  for (i=0; i<121; i++)
    for (j=0; j<3; j++) {
      if (strengths[j*121+i] < smin)
        smin = strengths[j*121+i];
      else if (strengths[j*121+i] > smax)
        smax = strengths[j*121+i];
      gamma = PI - radians(i - 60);
      d = mesures[i][j];
      l = sqrt(r * r + d * d - 2 * r * d * cos(gamma));
      alpha = asin(d * sin(gamma) / l) + j * TWO_PI / 3;
      //    println("g="+gamma+"/d="+d+"/l="+l+"/alpha="+alpha);
      points[j * 121 + i][0] = l * cos(alpha);
      points[j * 121 + i][1] = 500 - l * sin(alpha);
      //    println("("+points[j * 121 + i][0]+","+points[j * 121 + i][1]+")");
    }

  println("smin = " + smin + " / smax = " + smax);
  minXY();
  sortPoints();
  computeEnveloppe();
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////computeConcaveHull();

  colorMode(HSB, 360, 100, 100);
  background(255);
  translate(50, 50);
  strokeWeight(3);
  stroke(255, 0, 0);

  for (i=0; i<363; i++) {
    strokeWeight(10 /** (strengths[i] - smin) / (smax - smin)*//*);
    /*if (i==121)
      stroke(0, 255, 0);
    else if (i==242)
      stroke(0, 0, 255);*//*
    stroke(i, 100, 100);
    point((points[i][0] - xmin), (points[i][1] - ymin));
  }

  println("pile = " + pile);
  for (int i=0; i<=pile; i++) {
    stroke(0, 0, 0);
    strokeWeight(15);
    point(points[enveloppe[i]][0] - xmin, points[enveloppe[i]][1] - ymin);
    println(enveloppe[i]);
    strokeWeight(10);
    stroke(0,0,50);
    line(points[enveloppe[i]][0] - xmin, points[enveloppe[i]][1] - ymin,points[enveloppe[(i+1%pile)]][0] - xmin, points[enveloppe[(i+1%pile)]][1] - ymin);
  }

  strokeWeight(10);
  stroke(128);
  point(-xmin, -ymin);
  save("out/enveloppeConvexe.png");
  exit();
}*/

ArrayList<Points> points;
int[][] mesures;
FloatList angles;
int pile;
float xmin, ymin;
boolean convexe = false;
boolean pict = false;
int nbIter = 1;
boolean debug = false;
boolean algo1 = false;
int epsilon = 25;

void minXY() {
  int minID = 0;
  xmin = points.get(0).xCoord;
  ymin = points.get(0).yCoord;

  for (int i=1; i<points.size(); i++) {
    if(points.get(i).yCoord < ymin) {
      ymin = points.get(i).yCoord;
      minID = i;
    }
    if (points.get(i).xCoord < xmin) {
      xmin = points.get(i).xCoord;
    }
  }
  
  //println("minXY: "+ points.get(minID));
  Points tmp = points.get(minID);
  points.set(minID, points.get(0));
  points.set(0, tmp);
}

void sortPoints() {
  int j;
  angles.append(0);
  for (int i=1; i<points.size(); i++) {
    angles.append((points.get(i).xCoord - points.get(0).xCoord) /
      (points.get(i).yCoord - points.get(0).yCoord));

    j = 1;
    while ((j < i) && (angles.get(j) < angles.get(i)))
      j++;
    if (j != i)
      insertPointsAngles(i, j);
  }
  //println("angles: "+angles);
}

void insertPointsAngles(int i, int j){
  Points tmpP = points.get(i);
  points.remove(i);
  float tmpA = angles.get(i);
  angles.remove(i);

  points.add(j, tmpP);
  angles = new FloatList((float[])splice(angles.toArray(), tmpA, j));
}

float prodVec3(int a, int b, int c) {
  return (points.get(b).xCoord - points.get(a).xCoord) * (points.get(c).yCoord  - points.get(a).yCoord) -
  (points.get(c).xCoord - points.get(a).xCoord) * (points.get(b).yCoord - points.get(a).yCoord);
}

// Graham scan
int[] computeEnveloppe() {
  IntList enveloppe = new IntList();
  enveloppe.set(0, 0);
  enveloppe.set(1, 1);
  pile=1;
  for (int i = 2; i<points.size(); i++) {
    while ((pile >=1) && prodVec3(enveloppe.get(pile-1), enveloppe.get(pile), i) > 0){
      pile--;
    }
    pile++;
    enveloppe.set(pile, i);
  }
  //println("enveloppe: "+enveloppe);
  for(int i=enveloppe.size()-1; i>pile; i--){
    enveloppe.remove(i);
  }
  //println("enveloppe raccourci: "+enveloppe);
  return enveloppe.toArray();
}

ArrayList<Edge> sortEdgeHull(ArrayList<Edge> hull, Points min){
  ArrayList<Edge> sortedHull = new ArrayList<Edge>();
  //println(hull);
  for(Edge e: hull){
    if(e.point1.equals(min)){
      sortedHull.add(e);
      //println("found" + e);
      break;
    }
  }
  loop:
  for(int i=0; i<hull.size(); i++){
    for(Edge e: hull){
      if(sortedHull.get(i).point2.equals(e.point1)){
        sortedHull.add(e);
        //println("found" + e);
        continue loop;
      }
    }
    println("nothing found");
    break;
  }
  return sortedHull;
}

void setup() {
  size(1000, 1000);
  scale(0.25);
  BufferedReader log = createReader("../LaLigneRouge_v2_0/output/LIDAR.log");
  String line = null;
  angles = new FloatList();
  mesures = new int[120][3];
  try {
    while ((line = log.readLine()) != null) {
      String[] pieces = split(line, " -> ");
      String[] pieces2 = split(pieces[1], ",");
      if(int(pieces2[2]) == 0){
        continue;
      }
      int i = int(pieces2[0]);  //deviation angle record
      int j = int(pieces2[1]);  //lidar "id" who did that record
      int d = int(pieces2[2]);  //distance recorded
      mesures[i-1][j] = d;
    }
    log.close();
  } 
  catch (IOException e) {
    e.printStackTrace();
  }

  points = new ArrayList<Points>();
  float r = 18;

  for (int i=0; i<120; i++)
    for (int j=0; j<3; j++) {
      float gamma = PI - radians(i - 60);
      int d = mesures[i][j];
      float l = sqrt(r * r + d * d - 2 * r * d * cos(gamma));
      float alpha = asin(d * sin(gamma) / l) + j * TWO_PI / 3;
      points.add(new Points(l * cos(alpha), (- l * sin(alpha))*-1, i+degrees(j * TWO_PI / 3)));
    }

  minXY();
  sortPoints();

  colorMode(HSB, 360, 100, 100);
  background(255);
  translate(50, 50);
  strokeWeight(3);
  stroke(255, 0, 0);

  for (int i=0; i<360; i++) {
    strokeWeight(10);
    stroke(i, 100, 100);
    if(points.get(i).angle == 283){
      strokeWeight(50);
    }
    point((points.get(i).xCoord - xmin), (points.get(i).yCoord - ymin));
  }
  
  if(convexe & !pict){
    int[] enveloppe = computeEnveloppe();
    println("pile = " + pile);
    for (int i=0; i<=pile; i++) {
      stroke(0, 0, 0);
      strokeWeight(15);
      point(points.get(enveloppe[i]).xCoord - xmin, points.get(enveloppe[i]).yCoord - ymin);
      println("enveloppe["+i+"]: "+enveloppe[i]);
      strokeWeight(10);
      stroke(0, 0, 50);
      line(points.get(enveloppe[i]).xCoord - xmin, points.get(enveloppe[i]).yCoord - ymin, points.get(enveloppe[(i+1)%(pile+1)]).xCoord - xmin, points.get(enveloppe[(i+1)%(pile+1)]).yCoord - ymin);
    }
      
    strokeWeight(10);
    stroke(128);
    point(-xmin, -ymin);
    save("out/enveloppeConvexe.png");
  }else if(!convexe & !pict){
    //println(points);
    if(algo1){
      println("Computing Algo1");
      int before = millis();
      ArrayList<Points> enveloppe = computeConcaveHull(points, 3);
      int after = millis();
      println("time in millis: "+(after-before));
      println("taille enveloppe = "+ enveloppe.size());
      for(int i = 0; i<enveloppe.size(); i++){
        stroke(0, 0, 0);
        strokeWeight(15);
        point(enveloppe.get(i).xCoord-xmin, enveloppe.get(i).yCoord - ymin);
        //println("enveloppe["+i+"]: "+enveloppe.get(i));
        strokeWeight(10);
        stroke(0, 0, 50);
        line(enveloppe.get(i).xCoord-xmin, enveloppe.get(i).yCoord-ymin, enveloppe.get((i+1)%(enveloppe.size())).xCoord-xmin, enveloppe.get((i+1)%(enveloppe.size())).yCoord-ymin);
      }
      strokeWeight(10);
      stroke(128);
      point(-xmin, -ymin);
      save("out/enveloppeConcave.png");
    } else {
      println("Computing Algo2");
      int before = millis();
      ArrayList<Edge> concaveHull = computeConcaveHullFromConvexe(points, 2.5);
      int after = millis();
      println("time in millis: "+(after-before));
      println("taille enveloppe = "+ concaveHull.size());
      ArrayList<Edge> sortedHull = sortEdgeHull(concaveHull, points.get(0));
      ArrayList<Points> concaveHullPts = new ArrayList<Points>();
      for (Edge e : sortedHull) {
        concaveHullPts.add(e.point1);
      }
      for(int i=0; i<concaveHull.size(); i++){
        stroke(0, 0, 0);
        strokeWeight(15);
        point(concaveHull.get(i).point1.xCoord-xmin, concaveHull.get(i).point1.yCoord - ymin);
        point(concaveHull.get(i).point2.xCoord-xmin, concaveHull.get(i).point2.yCoord - ymin);
        strokeWeight(10);
        stroke(0, 0, 50);
        line(concaveHull.get(i).point1.xCoord-xmin, concaveHull.get(i).point1.yCoord-ymin, concaveHull.get(i).point2.xCoord-xmin, concaveHull.get(i).point2.yCoord-ymin);
      }
      Object[] hullDivided = hullDivider(concaveHullPts);
      ArrayList<Points> hull1 = (ArrayList<Points>)hullDivided[0];
      ArrayList<Points> hull2 = (ArrayList<Points>)hullDivided[1];
      ArrayList<Points> res1 = douglasPeucker(hull1, epsilon);
      ArrayList<Points> res2 = douglasPeucker(hull2, epsilon);
      for(int i=0; i<res1.size()-1; i++){
        stroke(240, 100, 100);
        strokeWeight(15);
        point(res1.get(i).xCoord-xmin, res1.get(i).yCoord - ymin);
        strokeWeight(10);
        stroke(240, 100, 100, 75);
        line(res1.get(i).xCoord-xmin, res1.get(i).yCoord-ymin, res1.get((i+1)%(res1.size())).xCoord-xmin, res1.get((i+1)%(res1.size())).yCoord-ymin);
      }
      for(int i=0; i<res2.size()-1; i++){
        stroke(0, 100, 100);
        strokeWeight(15);
        point(res2.get(i).xCoord-xmin, res2.get(i).yCoord - ymin);
        strokeWeight(10);
        stroke(0, 100, 100, 75);
        line(res2.get(i).xCoord-xmin, res2.get(i).yCoord-ymin, res2.get((i+1)%(res2.size())).xCoord-xmin, res2.get((i+1)%(res2.size())).yCoord-ymin);
      }
      println(concaveHullPts.size() + " points -> " + (res1.size() + res2.size() - 2) + " points");
      strokeWeight(20);
      stroke(0, 100, 100);
      point(-xmin, -ymin);
      save("out/enveloppeConcaveNew2.png");
    }
  }else{
    save("out/LidarPicture.png");
    println(points);
  }
  if(!debug){
    exit();
  }
}

//debug

void draw(){
  if(!debug){
    exit();
  }else{
    background(255);
    scale(0.25);
    translate(50, 50);
    strokeWeight(3);
    stroke(255, 0, 0);
    frameRate(6);

    for (int i=0; i<360; i++) {
      strokeWeight(10);
      stroke(i, 100, 100);
      point((points.get(i).xCoord - xmin), (points.get(i).yCoord - ymin));
    }

    ArrayList<Edge> concaveHull = computeConcaveHullFromConvexe(points, 2);
    println("taille enveloppe = "+ concaveHull.size());
    println(nbIter);
    for(int i=0; i<concaveHull.size(); i++){
      stroke(0, 0, 0);
      strokeWeight(15);
      point(concaveHull.get(i).point1.xCoord-xmin, concaveHull.get(i).point1.yCoord - ymin);
      point(concaveHull.get(i).point2.xCoord-xmin, concaveHull.get(i).point2.yCoord - ymin);
      strokeWeight(10);
      stroke(0, 0, 50);
      line(concaveHull.get(i).point1.xCoord-xmin, concaveHull.get(i).point1.yCoord-ymin, concaveHull.get(i).point2.xCoord-xmin, concaveHull.get(i).point2.yCoord-ymin);
    }
    strokeWeight(10);
    stroke(128);
    point(-xmin, -ymin);
  }
}

void keyPressed() {
  if (key == ' ')
    nbIter++;
}
