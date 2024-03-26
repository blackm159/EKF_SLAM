# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
      1) Take a vector as an input, and output that vector after by dividing the elements by the magnitude
      2) Overload the / operator to perform element operations for vectors.  Use this to output a normalized vector
      3) Divide the elements of the vector by its magnitude in the main script

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
      1) **Pros:** This is a mostly quick implementation and only provides the desired operation.  **Cons:** This function would overwrite the original vector
      2) **Pros:** This also allows for vector scaling, which could be useful.  **Cons:** Adds an additional function that may not be necessary.
      3) **Pros:** This is a quick implementation.  **Cons:** Normalizing a vector is something that may need to happen more than once and implementing each time would be redundant.

   - Which of the methods would you implement and why?

      - I would implement the second method because it gains additional functionality with only a small amount of effort more.

2. What is the difference between a class and a struct in C++?

   - A class is private by default, while a struct is public by default.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

   - Vector2D is a struct because it is a simple data structure that does not require any additional functionality.  Transform2D is a class because it is a more complex data structure that requires additional functionality.  C.2 shows that structs are public facing, while classes have private members.  C.3 shows that it is important to keep an implementation seperate from an interface, which is why Transform2D is a class.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

   - The constructors that take a single argument are explicit because they are not intended to be used for implicit conversions.  C.46 shows that explicit constructors should be used to avoid accidental conversions.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

      - Transform2D::inv() is declared const because it does not modify the object.  Transform2D::operator*=() is not declared const because it modifies the object.  Con.1 shows that const should be used to indicate that a function does not modify an object.