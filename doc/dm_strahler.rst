=================
Strahler Ordering
=================

Calculates the `Strahler order <https://en.wikipedia.org/wiki/Strahler_number>`_ for each edge in a directed graph. It is recommended to direct the graph with :doc:`DM_DirectGraph </DynaMind-GDALModules/dm_direct_graph>`.

Parameter
---------

+-----------------------+------------------------+------------------------------------------------------------------------+
|        Name           |          Type          |       Description                                                      |
+=======================+========================+========================================================================+
|view_name              | STRING                 | lead view name                                                         |
+-----------------------+------------------------+------------------------------------------------------------------------+
|root_node              | INT                    | id of the root node                                                    |
+-----------------------+------------------------+------------------------------------------------------------------------+


Data-stream
-----------

+--------------------+---------------------------+------------------+-------+------------------------------------------+
|        View        |          Attribute        |       Type       |Access |    Description                           |
+====================+===========================+==================+=======+==========================================+
| view_name          |                           | EDGE             | read  |                                          |
+--------------------+---------------------------+------------------+-------+------------------------------------------+
|                    | start_id                  | INT              | read  | downstream node                          |
+--------------------+---------------------------+------------------+-------+------------------------------------------+
|                    | end_id                    | INT              | read  | upstream node                            |
+--------------------+---------------------------+------------------+-------+------------------------------------------+
|                    | strahler_order            | INT              | write | strahler order                           |
+--------------------+---------------------------+------------------+-------+------------------------------------------+

Module Name
-----------

:index:`DM_Strahler <GDALModules; DM_Strahler | Strahler ordering>`