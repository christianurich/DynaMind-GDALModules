=========================
Generate Simple Buildings
=========================

 Generates simple
buildings in the centre of the parcel


Parameter
---------

+-------------------+------------------------+------------------------------------------------------------------------+
|        Name       |          Type          |       Description                                                      |
+===================+========================+========================================================================+
|width              | DOUBLE                 | standard with of the building                                          |
+-------------------+------------------------+------------------------------------------------------------------------+
|length             | DOUBLE                 | standard length of the building                                        |
+-------------------+------------------------+------------------------------------------------------------------------+
|residential_units  | INT                    | residential units                                                      |
+-------------------+------------------------+------------------------------------------------------------------------+


Data-stream
-----------

+---------------------+--------------------------+------------------+-------+------------------------------------------+
|        View         |          Attribute       |       Type       |Access |    Description                           |
+=====================+==========================+==================+=======+==========================================+
|   parcel            |                          | FACE             | read  | parcels where building is placed         |
+---------------------+--------------------------+------------------+-------+------------------------------------------+
|                     |                          |                  |       |                                          |
+---------------------+--------------------------+------------------+-------+------------------------------------------+
|   building          |                          | FACE             | write | resulting faces                          |
+---------------------+--------------------------+------------------+-------+------------------------------------------+
|                     |  residential_units       | INT              | write | residential units (from input)           |
+---------------------+--------------------------+------------------+-------+------------------------------------------+


Module Name
-----------

:index:`GDALCreateBuilding <GDALModules; GDALCreateBuilding | Generate Simple Buildings>`


Detailed Description
--------------------

Creates simple buildings in the centroids of the parcel.

