
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
      if(sortedHull.get(i).point2.equals(e.point1) && !sortedHull.contains(e)){
        sortedHull.add(e);
        //println("found" + e);
        continue loop;
      }
    }
    //println("nothing found");
    break;
  }
  return sortedHull;
}

//Fonction calcul
ArrayList<Points> main(String logPath, String name, float xdecal, float ydecal, float adecal) {
  BufferedReader log = createReader(logPath);
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
  float r = 20;

  for (int i=0; i<120; i++)
    for (int j=0; j<3; j++) {
      float gamma = radians(i - 60);
      int d = mesures[i][j];
      float l = sqrt(r * r + d * d + 2 * r * d * cos(gamma));
      float alpha = asin((d * sin(gamma)) / l) + ((PI/3)+j * TWO_PI / 3)+ radians(adecal);
      points.add(new Points(l * cos(alpha)+xdecal, (- l * sin(alpha))*-1+ydecal, i+degrees(j * TWO_PI / 3)));
    }

  minXY();
  sortPoints();

  colorMode(HSB, 360, 100, 100);
  background(#ffffff);
  strokeWeight(3);
  stroke(255, 0, 0);

  for (int i=0; i<360; i++) {
    strokeWeight(10);
    stroke(0, 0, 0);
    /*
    if(points.get(i).angle == 350){
      strokeWeight(50);
    }
    */
    point((points.get(i).xCoord - xmin), (points.get(i).yCoord - ymin));
  }
  strokeWeight(20);
  stroke(0, 100, 100);
  point(-xmin, -ymin);
  
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
      ArrayList<Points> concaveHullPts = new ArrayList<Points>();
      for (Edge e : concaveHull) {
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
      println(concaveHullPts.size() + " points -> " + (res1.size() + res2.size()) + " points");
      concaveHullPts.clear();
      concaveHullPts.addAll(res1);
      for(Points p: res2){
        if(!concaveHullPts.contains(p)){
          concaveHullPts.add(p);
        }
      }
      ArrayList<Points> pointL = new ArrayList<Points>();
      for(int i=0; i<concaveHullPts.size(); i++){
        float[] e1 = concaveHullPts.get(i).vectorize(concaveHullPts.get((i-1)<0?concaveHullPts.size()-1:i-1));
        float[] e2 = concaveHullPts.get(i).vectorize(concaveHullPts.get((i+1)%concaveHullPts.size()));
        float prodScal = e1[0]*e2[0]+e1[1]*e2[1];
        float e1Norm = sqrt(sq(e1[0])+sq(e1[1]));
        float e2Norm = sqrt(sq(e2[0])+sq(e2[1]));
        float angle = degrees(acos(prodScal/(e1Norm*e2Norm)));
        //println(concaveHullPts.get(i) +" : "+angle);
        if(angle>=45 && angle<=135){
          strokeWeight(40);
          stroke(0, 0, 0);
          point(concaveHullPts.get(i).xCoord-xmin, concaveHullPts.get(i).yCoord-ymin);
          pointL.add(concaveHullPts.get(i));
        }
      }

      after = millis();
      println("time in millis global: "+(after-before));
      save("out/" + name +".png");
      return pointL;
    }
  }else{
    save("out/LidarPicture.png");
    println(points);
  }
  return null;
}


void setup(){
  size(1000, 1000);
  scale(0.25);
  translate(100, 75);
  if(convexe || pict || algo1){
    main("../LaLigneRouge_v2_0/output/LIDAR1.log", "enveloppeConcaveNew", 0, 0, 0);
    return;
  }
  int t = millis();
  // Calcul des enveloppes des deux scans et récupère la liste des points importants
  ArrayList<Points> array2 = main("../LaLigneRouge_v2_0/output/LIDAR2.log", "enveloppeConcaveNew2", 0, 0, 260); //mesure après déplacement (ajouter rotation en troisième position)
  ArrayList<Points> array1 = main("../LaLigneRouge_v2_0/output/LIDAR1.log", "enveloppeConcaveNew", 0, 0, 0);
  println("array1: " + array1);
  println("array2: " + array2);

  // Calcul des transformations pour passer d'une enveloppe à l'autre 
  float[][] data = clustering(array1, array2);
  ArrayList<float[]> dataList = new ArrayList<>();
  for (float[] element : data) {
    if (element[1] >= 0.9 && element[1] <= 1.1) {
      dataList.add(element);
    }
  }
  data = dataList.toArray(new float[dataList.size()][]);
  println("data after filter: ");
  for(float[] element : data){
    int i = 0;
    for(float value : element){
      print("["+i+"] "+value + " ");
      i++;
    }
    println();
  }
  // Utilisation de l'algo de clustering pour retrouver la transformation globale entre nos deux enveloppes
  int[] cluster = dbscan(data, 60, 2);
  println("cluster: ");
  for(int element : cluster){
    print(element + " ");
  }
  println();
  int n = max(cluster);
  if(n<0){
    println("Aucune transformation n'apparait plus de deux fois");
    return;
  }
  int[] clusterCount = new int[n];
  float[] thetaG = new float[n];
  float[] sinThetaG = new float[n];
  float[] cosThetaG = new float[n];
  float[] deltaXG = new float[n];
  float[] deltaYG = new float[n];

  for (int i = 0; i < cluster.length; i++) {
    int clusterIndex = cluster[i] - 1;
    if (clusterIndex >= 0 && clusterIndex < n) {
      clusterCount[clusterIndex]++;
      sinThetaG[clusterIndex] += sin(radians(data[i][0]));
      cosThetaG[clusterIndex] += cos(radians(data[i][0]));
      deltaXG[clusterIndex] += data[i][2];
      deltaYG[clusterIndex] += data[i][3];
    }
  }

  for (int i = 0; i < n; i++) {
    if (clusterCount[i] > 0) {
      float avgSin = sinThetaG[i]/clusterCount[i];
      float avgCos = cosThetaG[i]/clusterCount[i];
      thetaG[i] = degrees(atan2(avgSin,avgCos));
      deltaXG[i] /= clusterCount[i];
      deltaYG[i] /= clusterCount[i];
    }
  }

  int clusterMax = 0;
  int transfo = -1;
  // Affichage des transformations moyennes obtenues dans chaque cluster
  for (int i = 0; i < n; i++) {
    println("Cluster " + (i + 1) + ":");
    println("  Count: " + clusterCount[i]);
    println("  Average theta: " + thetaG[i]);
    println("  Average deltaX: " + deltaXG[i]);
    println("  Average deltaY: " + deltaYG[i]);
    if(clusterMax<clusterCount[i]){
      clusterMax = clusterCount[i];
      transfo = i;
    }
  }

  // Affichage du cluster possédant la plus grande population
  println("Cluster " + (transfo + 1) + ":");
  println("  Count: " + clusterCount[transfo]);
  println("  Average theta: " + thetaG[transfo]);
  println("  Average deltaX: " + deltaXG[transfo]);
  println("  Average deltaY: " + deltaYG[transfo]);
  println("time in millis: "+(millis()-t));
}

// fonction utilisée en cas de debug
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