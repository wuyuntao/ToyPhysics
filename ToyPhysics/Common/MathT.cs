﻿namespace ToyPhysics
{
    static class MathT
    {
        public static void Swap<T>(ref T t1, ref T t2)
        {
            var temp = t1;
            t1 = t2;
            t2 = temp;
        }
    }
}
