
class Map {
  private int width;
  private int height;
  private String name;
  //PVector<PVector<Integer>> board;
  public int[][] board;

  Map( String filename ) {
    String[] lines = loadStrings("Map/" + filename );
    height = lines.length;
    width = 0;
    
    //calculate max width of csv file
    for (int i=0; i < height; i++) {
      String [] chars=split(lines[i],',');
      if (chars.length>width){
        width=chars.length;
      }
    }
    
    // create map array
    board = new int[height][width];
    for( int i=0; i < height; i++ ){
      String[] tmp = new String[width];
      tmp = split(lines[i], ',');
      for( int j=0; j < width; j++ ){
        board[i][j] = int(tmp[j]);
      }
    }
    
    for( int i=0; i < height; i++ ){
      for( int j=0; j < width; j++ ){
        System.out.print( board[i][j] + " " );
      }
      System.out.println();
    }
  }
}
