
class Map {
  private int xlength;
  private int ylength;
  private String name;
  //PVector<PVector<Integer>> board;
  public int[][] board;

  Map( String filename ) {
    String[] lines = loadStrings("Map/" + filename );
    ylength = lines.length;
    xlength = 0;
    
    //calculate max width of csv file
    for (int i=0; i < ylength; i++) {
      String [] chars=split(lines[i],',');
      if (chars.length>xlength){
        xlength=chars.length;
      }
    }
    
    // create map array
    board = new int[ylength][xlength];
    for( int i=0; i < ylength; i++ ){
      String[] tmp = new String[xlength];
      tmp = split(lines[i], ',');
      for( int j=0; j < xlength; j++ ){
        board[i][j] = int(tmp[j]);
      }
    }
    
    for( int i=0; i < ylength; i++ ){
      for( int j=0; j < xlength; j++ ){
        System.out.print( board[i][j] + " " );
      }
      System.out.println();
    }
  }
  
  public int getXLength(){
    return xlength;
  }
  
  public int getYLength(){
    return ylength;
  }
}
