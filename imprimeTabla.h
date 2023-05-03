public void imprimeTabla(int idNodoEsclavo_1,int idNodoEsclavo_2, int rssi)
{
    // Imprimir encabezado de tabla
    console.WriteLine("La siguiente tabla muestra el RSSI que hay entre el Nodo Esclavo 1 y el 2")
    Console.WriteLine("| ID Nodo Esclavo 1 |ID Nodo Esclavo 2 |  RSSI  |");
    Console.WriteLine("|-------------------|------------------|--------|");
    
    // Imprimir valores de tabla
    //Los valores 0 y 1 se refieren al argumento que se imprimira en esa posición, idNodoEsclavo (0) y rssi(1)
    //El -15 y el -8 son la "anchura" que tendrá ese argumento y el "-" significa que estará alienado a la izquierda
    Console.WriteLine("| {0, -15} | {1, -15} | {2, -8}", idNodoEsclavo_1,idNodoEsclavo_2 rssi);
    
    if( idNodoEsclavo_1 <= 2 && idNodoEsclavo_2 <= 2)
    {
        //Esta linea me imprimiria la id del nodo en azul y el rssi en rojo. Podria hacer que segun los id, me imprimiera su valor de un color u otro.
        //Al final pongo "\u001b[0m" para que reestablezca el valor del color para que todo lo demas no se imprima de ese color.
        Console.WriteLine("| \u001b[34m{0, -15}\u001b[0m | \u001b[34m{1, -15}\u001b[0m | \u001b[31m{2, -8}\u001b[0m |", idNodoEsclavo_1, idNodoEsclavo_2, rssi);
    }
    else
    { 
        //Si el nodo tiene id 3 o 4 imprimira su id de color verde y su rssi de color morado
        Console.WriteLine("| \u001b[32m{0, -15}\u001b[0m | \u001b[32m{1, -15}\u001b[0m | \u001b[35m{2, -8}\u001b[0m |", idNodoEsclavo_1, idNodoEsclavo_2, rssi);
    }
    
    
    
    // No es necesario devolver nada, ya que el método es "void"
    
}





