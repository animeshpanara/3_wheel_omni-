
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
{
  for(int x = 0; x < 3; x++)
  {
    for(int y = 0; y < 3; y++)
    {
      mat[x][y] = 0;
      
      for(int w = 0; w < 3; w++)
      {
        mat[x][y] += a[x][w] * b [w][y];
      } 
    }
  }
}

