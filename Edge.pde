class Edge {
  Points point1;
  Points point2;

  Edge(Points p1, Points p2) {
    point1 = p1;
    point2 = p2;
  }

  boolean equals(Edge e) {
    return (point1.equals(e.point1) && point2.equals(e.point2)) || (point1.equals(e.point2) && point2.equals(e.point1));
  }

  float distance() {
    return point1.distance(point2);
  }

  @Override
  String toString(){
      return "[" + point1 + ", " + point2 + "]";
  }
}
