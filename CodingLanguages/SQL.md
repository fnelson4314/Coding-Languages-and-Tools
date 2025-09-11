# SQL

### Databases

- There are two types of databases:
  - Relational Databases(SQL) organize data into one or more tables with each table having columns and rows as well as a unique key identifying each row. This is like an Excel spreadsheet
    - Can be used with Relational Database Management Systems(RDBMS) such as mySQL, Oracle, postgreSQL, mariaDB, etc.
  - Non-Relational (noSQL) organizes data in anything but a traditional table. This can be with key-value stores, documents like (JSON, XML, etc.), graphs, and flexible tables
    - Can be used with mongoDB, dynamoDB, apache cassandra, firebase, etc.

### Variable Types

- INT - Whole Numbers/integers
- DECIMAL(m, n) - Decimal numbers
  - The m is the total number of decimal places
  - The n is the number of decimal places after the decimal point
- VARCHAR(l) - This is for strings
  - l is how many chars you want your string to be
- BLOB - Binary large objects
- DATE - ‘YYYY-MM-DD’
- TIMESTAMP - ‘YYYY-MM-DD HH:MM:SS’

### Creating/Modifying Data Tables

- Primary Keys are the unique keys that distinguish the different rows - For example, if some data in the rows were exactly the same, you need to have a way to distinguish between the two so this is where unique keys come in
  - ![redTable](images/redTable.png) The primary keys are in red
  - The column in green is called the foreign key which links us to the primary key of another table
- Syntax: **CREATE TABLE tableName(tableData);**
```sql
CREATE TABLE student(
  student_id INT PRIMARY KEY,
  name VARCHAR(20),
  major VARCHAR(20)
);
```
- Instead of putting PRIMARY KEY right after the field, you can put it all the way down at the bottom like **PRIMARY KEY(student_id)**. Can also do two fields like **PRIMARY KEY(student_id, major)** for a composite key
- To delete a table, simply do **DROP TABLE tableName;**
- **ALTER** keyword allows you to add, delete, or modify columns in a table
- To add another column to an existing table, do **ALTER TABLE tableName ADD columnName columnType;**
- You can also drop a column using **ALTER TABLE tableName DROP COLUMN columnName**
- You can introduce a foreign key like **FOREIGN KEY(fieldName) REFERENCES foreignTableName(foreign field being referenced)**

### Inserting Data

- To insert data into an existing table, do **INSERT INTO tableName(columnNames of what you want required(you don’t have to put these column names if you want all of them))VALUES(values);**
```sql
INSERT INTO student(student_id, name) VALUES
(1, ‘Jack’, ‘Biology’),
(2, ‘Kate’, ‘Sociology’);
```

### Constraints

- Constraints can be added at the end of a field when adding one to a new table
- **NOT NULL** can be added after any field when creating a table to make sure that a value ALWAYS has to be there when inserting data
  - Ex. **name VARCHAR(20) NOT NULL;**
- **UNIQUE** can be added after any field when creating a table to make sure that each value inserted is unique
  - Ex. **major VARCHAR(20) UNIQUE;**
- **DEFAULT** is used to specify a default value if one is not given
  - Ex. **major VARCHAR(20) DEFAULT ‘undecided’;**
- **AUTO_INCREMENT** can be added to automatically increment instead of you having to do it yourself when inserting data
  - Ex. **student_id INT AUTO_INCREMENT;**

### Update and Delete Rows

- UPDATE and DELETE can be used to modify rows
- Update using **UPDATE tableName**
```sql
UPDATE student
SET major = 'Bio', name = 'John'
WHERE major = 'Biology'
```
- Delete using **DELETE FROM tableName;** which would delete all the rows in tableName
```sql
DELETE FROM student
WHERE student_id = 5;
```

### Keyword Library

![Keyword order table](images/order.jpg)

- **SELECT** - Allows you to select certain data from a database
  - To see the entire table, you can do **SELECT \***
  - For more exact do **SELECT columnName, …**
- **DISTINCT** - Used to return only distinct values
  - **SELECT DISTINCT major**
- **WHERE** - The filter word for records. Extracts only certain data that satisfies a condition
  - **IN** - Allows you to specify multiple values
    - **WHERE column\*name IN (value1, value2, ...);**
  - **Conditionals(AND, OR, NOT)** - Same as in other languages
    - Other conditionals: **<, >, <=, >=, =, <>(Not equal)**
  - **LIKE** - Search for a specified pattern in a column
    - % sign represents zero, one, or multiple characters
      - Ex. **WHERE CustomerName LIKE ‘a%’;**
    - \_ sign represents one, single character
      - Ex. **WHERE city LIKE ‘L_nd\_\_’;**
  - **BETWEEN** - Used to look for a result between two values
    - Ex. **WHERE year BETWEEN 1900 and 1950;**
- **FROM** - Used to specify which table to select or delete data from
- **GROUP BY** - Groups rows that have the same values into summary rows
  - Usually used with aggregate functions **(COUNT(), MAX(), MIN(), SUM(), AVG())**
  - ```sql
    SELECT COUNT(CustomerID), Country
    FROM Customers
    GROUP BY Country; -- This will return how many of each country there is
    ```
- **ORDER BY** - Used to sort the result-set in ascending or descending order
  - Add **ASC** at the end for it to be in ascending order(default)
  - Add **DESC** at the end for descending order
  - Can use column indexes instead of full variable names(SQL is 1-indexed)
- **HAVING** - Same thing as the WHERE keyword but it works with aggregate functions since WHERE can’t
  - Goes after the GROUP BY keyword
- **LIMIT** - Sets the number of rows that you want outputted
  - **Takes in two parameters: Ex. **LIMIT amount_of_rows, next_row_after_last\*\*
- **AS** - Can go after any variable to change its name to something else
  - Ex. **AVG(age) AS avg_age**
  - Can also do like this and it will produce the same result: **AVG(age) avg_age**
- JOIN - Used to combine rows from two or more tables based on a related column
  - **ON** - Used beneath the INNER JOIN clause where you decide what is equal
  - **(INNER) JOIN** - Returns recorder matching in both tables
  - **LEFT (OUTER) JOIN** - Returns all from left table(FROM clause) and matches from right
  - **RIGHT (OUTER) JOIN** - Same as left but reverse
  - **FULL (OUTER) JOIN** - Returns all records when there is a match in either table
- **UNION** - Combines rows using multiple SELECT statements
  - Allows only distinct values by default
  - Can do **UNION ALL** if you want to allow duplicate values
  - Syntax:
    - **SELECT column_name(s) FROM table1**
    - **UNION ALL**
    - **SELECT column_name(s) FROM table2;**
- **UPPER( str )** - Changes to all uppercase
- **LOWER( str )** - Changes to all lowercase
- **TRIM( str )** - Removes whitespace from leading and trailing spaces
  - **LTRIM( str )** - Removes whitespace from leading spaces
  - **RTRIM( str )** - Removes whitespace from trailing spaces
- **SUBSTRING(var, start, n)** - Grabs the characters starting from start and grabs the next n characters(including start). Ex. Leslie -> **SUBSTRING(first_name,3,2)** -> sl
  - **Left(var, n)** - Only grabs the first n characters from the left
  - **RIGHT(var, n)** - Only grabs the last n characters from the right
- **REPLACE(var, str_search, str_replace)** - Searches for a string then replaces it
- **LOCATE(search, str/var)** - Searches for the index of the character in the string/variable
- **CONCAT(var1, var2, var3, …)** - Combines row data into one string
- **CASE** - Like an if else statement, goes through conditions and returns when first condition met
  - **CASE**
  - **WHEN condition1 THEN result1**
  - **WHEN condition2 THEN result2**
  - **WHEN conditionN THEN resultN**
  - **ELSE result**
  - **END;**
    - Can also use **BETWEEN** to find things between two values
    - Ex. **BETWEEN 31 AND 50**

### Window Functions

- **OVER(PARTITION BY col_name)** - Substitute for GROUP BY so that you’re data can be more separate
  - Ex. **SELECT AVG(salary) OVER(PARTITION BY gender ORDER BY employee.id)**
- **ROW_NUMBER()** - Gets the row number and is usually used with OVER()
- **RANK()** - Like row number but it doesn’t allow duplicates and it’ll give duplicates the same rank
- **DENSE_RANK()** - Different from RANK() since after duplicates it’ll go to the next highest number instead of the normal row number it should be like in RANK()

### CTE (Common Table Expression)

- Basically just a prettier version of a subquery since the name is at the top and it’s more organized
- Can only use your CTE immediately after you create it
- Is created using the WITH keyword: Ex. **WITH CTE_example AS (subquery)**
