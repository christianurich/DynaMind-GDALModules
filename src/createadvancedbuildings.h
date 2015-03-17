#ifndef CREATEADVANCEDBUILDINGS_H
#define CREATEADVANCEDBUILDINGS_H

#include <dmmodule.h>
#include <dm.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/extremal_polygon_2.h>

#include <SFCGAL/MultiPolygon.h>
#include <SFCGAL/MultiLineString.h>

#include <SFCGAL/io/wkt.h>
#include <SFCGAL/algorithm/offset.h>
#include <SFCGAL/algorithm/difference.h>
#include <SFCGAL/algorithm/intersection.h>
#include <SFCGAL/algorithm/convexHull.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Gps_circle_segment_traits_2.h>
#include <CGAL/approximated_offset_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include <SFCGAL/MultiPolygon.h>
#include <SFCGAL/io/wkt.h>

typedef CGAL::Point_2< SFCGAL::Kernel >              Point_2 ;
typedef CGAL::Vector_2< SFCGAL::Kernel >             Vector_2 ;
typedef CGAL::Segment_2< SFCGAL::Kernel >               Segment_2 ;
typedef CGAL::Polygon_2< SFCGAL::Kernel >            Polygon_2 ;
//typedef CGAL::Polygon_with_holes_2< SFCGAL::Kernel > Polygon_with_holes_2;
//typedef std::list<Polygon_with_holes_2>              Pwh_list_2;
typedef CGAL::Gps_circle_segment_traits_2<SFCGAL::Kernel>	Gps_traits_2;
typedef Gps_traits_2::Polygon_2								Offset_polygon_2;


class DM_HELPER_DLL_EXPORT OGRPolygon;

class DM_HELPER_DLL_EXPORT CreateAdvancedBuildings : public DM::Module
{
		DM_DECLARE_NODE(CreateAdvancedBuildings)
private:

	DM::ViewContainer parcel;
	DM::ViewContainer building;
	DM::ViewContainer cityblock;
	DM::ViewContainer street_access;

	double width;
	double height;
	int residential_units;
	OGRGeometry *createBuilding(OGRPolygon *ogr_poly, OGRPolygon *ogr_boundary);
	OGRGeometry *addToSystem(SFCGAL::Polygon &poly);
	Polygon_2 approximate(const Offset_polygon_2 &polygon, const int &n = 0);
	std::list<Polygon_2> offsetPolygon(Polygon_2 poly, double offset);
public:
	CreateAdvancedBuildings();
	void run();


};


#endif // CREATEADVANCEDBUILDINGS_H
