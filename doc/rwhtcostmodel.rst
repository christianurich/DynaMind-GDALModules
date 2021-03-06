=============
RWHTCostModel
=============

Assesses construction and costs and water savings for the installation of rain water harvesting tanks.
To assess the construction costs data from `Tim et. al. 2010 <http://www.sciencedirect.com/science/article/pii/S0921344909001621>`_ and
for water savings is based on data from `South East Water <http://southeastwater.com.au/Residential/Pages/WaterPricesCharges.aspx>`_ are used.

Data-stream
-----------

+-------------------+--------------------------+------------------+-------+----------------------------------------------------------------------------------------------------+
|        View       |          Attribute       |       Type       |Access |    Description                                                                                     |
+===================+==========================+==================+=======+====================================================================================================+
| **rwht**          |                          | COMPONENT        | write | rain water harvesting tank                                                                         |
+-------------------+--------------------------+------------------+-------+----------------------------------------------------------------------------------------------------+
|                   | volume                   |    DOUBLE        | read  | tank volume (m3)                                                                                   |
+-------------------+--------------------------+------------------+-------+----------------------------------------------------------------------------------------------------+
|                   | annual_water_savings     |    DOUBLE        | read  | water in tank (m3/day)                                                                             |
+-------------------+--------------------------+------------------+-------+----------------------------------------------------------------------------------------------------+
|                   | construction_costs       |    DOUBLE        | write | $AUD                                                                                               |
+-------------------+--------------------------+------------------+-------+----------------------------------------------------------------------------------------------------+
|                   | annual_savings           |    DOUBLE        | write | $AUD                                                                                               |                               
+-------------------+--------------------------+------------------+-------+----------------------------------------------------------------------------------------------------+
|                   | maintenance_costs        |    DOUBLEVECTOR  | write | no included yet                                                                                    |
+-------------------+--------------------------+------------------+-------+----------------------------------------------------------------------------------------------------+
