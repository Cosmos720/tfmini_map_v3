float[][] clustering(ArrayList<Points> modelPoints, ArrayList<Points> scenePoints){

  ArrayList<float[]> dataTransform = new ArrayList<float[]>();
  /*for(int i=0; i<modelPoints.size(); i++){
    for(int j=0; j<scenePoints.size(); j++){
      dataTransform[i*scenePoints.size()+j] = transformCalcul(modelPoints.get(i), modelPoints.get((i+1)%modelPoints.size()), scenePoints.get(j), scenePoints.get((j+1)%scenePoints.size()));
      print("("+i+", "+j+")"+"[a] "+dataTransform[i*scenePoints.size()+j][0] + " [s] " + dataTransform[i*scenePoints.size()+j][1] + " [x] " + dataTransform[i*scenePoints.size()+j][2] + " [y] " + dataTransform[i*scenePoints.size()+j][3] + "\n");
    }
  }*/
  for(Points mi: modelPoints){
    for(Points mj: modelPoints){
      if(mj.equals(mi)){
        continue;
      }
      for(Points si: scenePoints){
        for(Points sj: scenePoints){
          if(sj.equals(si)){
            continue;
          }
          dataTransform.add(transformCalcul(mi, mj, si ,sj));
        }
      }
    }
  }
  return dataTransform.toArray(new float[dataTransform.size()][]);
}

float[] transformCalcul(Points modelA, Points modelB, Points sceneA, Points sceneB){
  float[] modelVector = modelA.vectorize(modelB);
  float[] sceneVector = sceneA.vectorize(sceneB);

  float theta = ((atan2(modelVector[1], modelVector[0]) - atan2(sceneVector[1],sceneVector[0]))+TWO_PI)%TWO_PI;

  float scale = modelA.distance(modelB)/sceneA.distance(sceneB);

  float deltaX = scale*sceneA.yCoord*sin(theta)-scale*sceneA.xCoord*cos(theta)+modelA.xCoord;
  float deltaY = -scale*sceneA.xCoord*sin(theta)-scale*sceneA.yCoord*cos(theta)+modelA.yCoord;

  float[] transform = {degrees(theta), scale, deltaX, deltaY};
  return transform;
}

int[] dbscan(float[][] data, float eps, int minPts){
  int[] cluster = new int[data.length];
  int clusterCount = 0;
  for(int i=0; i<data.length; i++){
    if(cluster[i] == 0){
      ArrayList<Integer> neighbors = findNeighbors(data, i, eps);
      
      if(neighbors.size() < minPts){
        cluster[i] = -1;
      } else {
        clusterCount++;
        expandCluster(data, cluster, i, neighbors, clusterCount, eps, minPts);
      }
    }
  }
  return cluster;
}

ArrayList<Integer> findNeighbors(float[][] data, int pointIndex, float eps){
  ArrayList<Integer> neighbors = new ArrayList<Integer>();
  
  for(int i=0; i<data.length; i++){
    if(i != pointIndex){
      float distance = calculateDistance(data[pointIndex], data[i]);
      if(distance <= eps){
        neighbors.add(i);
      }
    }
  }
  
  return neighbors;
}

void expandCluster(float[][] data, int[] cluster, int pointIndex, ArrayList<Integer> neighbors, int clusterCount, float eps, int minPts){
  cluster[pointIndex] = clusterCount;
  
  for(int i=0; i<neighbors.size(); i++){
    int neighborIndex = neighbors.get(i);
    
    if(cluster[neighborIndex] == 0){
      ArrayList<Integer> neighborNeighbors = findNeighbors(data, neighborIndex, eps);
      
      if(neighborNeighbors.size() >= minPts){
        neighbors.addAll(neighborNeighbors);
      }
    }
    
    if(cluster[neighborIndex] <= 0){
      cluster[neighborIndex] = clusterCount;
    }
  }
}

float calculateDistance(float[] pointA, float[] pointB){
  return sqrt(pow(pointA[0]-pointB[0], 2) + pow(pointA[2]-pointB[2], 2) + pow(pointA[3]-pointB[3], 2));
}