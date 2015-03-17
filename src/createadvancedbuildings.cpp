#include "createadvancedbuildings.h"

#include <ogr_api.h>
#include <ogrsf_frmts.h>
#include <CGAL/Aff_transformation_2.h>

typedef CGAL::Point_2< SFCGAL::Kernel >              Point_2 ;
typedef CGAL::Vector_2< SFCGAL::Kernel >             Vector_2 ;
typedef CGAL::Polygon_2< SFCGAL::Kernel >            Polygon_2 ;
typedef CGAL::Polygon_with_holes_2< SFCGAL::Kernel > Polygon_with_holes_2;
typedef std::list<Polygon_with_holes_2>              Pwh_list_2;

DM_DECLARE_NODE_NAME(CreateAdvancedBuildings, GDALModules)


std::list<Polygon_2> CreateAdvancedBuildings::offsetPolygon(Polygon_2 poly, double offset)  {
	typedef CGAL::Gps_circle_segment_traits_2<SFCGAL::Kernel>  Gps_traits_2;
	typedef Gps_traits_2::Polygon_2                            Offset_polygon_2;
	std::list<Polygon_2> offset_polygons;
	if(!poly.is_simple()) {
		DM::Logger(DM::Warning) << "Can't perform offset polygon is not simple";
		return offset_polygons;
	}
	CGAL::Orientation orient = poly.orientation();
	if (orient == CGAL::CLOCKWISE) {
		poly.reverse_orientation();
	}

	const double                           err_bound = 0.00001;
	std::list<Offset_polygon_2>            inset_polygons;

	CGAL::approximated_inset_2 (poly, offset, err_bound, std::back_inserter (inset_polygons));
	std::list<Polygon_2> p_list;
	foreach (Offset_polygon_2 p, inset_polygons) {
		p_list.insert(p_list.begin(), this->approximate(p));
	}
	return p_list;
}

Polygon_2 CreateAdvancedBuildings::approximate( const Offset_polygon_2& polygon, const int& n )
{
	std::list<std::pair<double, double> > pair_list;

	/*
	 * iterate X_monotone_curve_2 components
	 */
	for ( Offset_polygon_2::Curve_const_iterator it = polygon.curves_begin();
		  it != polygon.curves_end(); ++it ) {
		it->approximate( std::back_inserter( pair_list ), n ) ;
	}

	// remove duplicated last point
	pair_list.pop_back() ;

	/*
	 * convertr to polygon
	 */
	Polygon_2 result ;

	bool isFirst = true ;
	SFCGAL::Kernel::Point_2 last ;
	SFCGAL::Kernel::Point_2 first ;

	for ( std::list<std::pair<double, double> >::const_iterator it = pair_list.begin(); it != pair_list.end(); ++it ) {
		SFCGAL::Kernel::Point_2 point( it->first, it->second ) ;

		if ( isFirst ) {
			first = point;
			isFirst = false ;
		}
		else if ( point == last ) {
			continue ;
		}

		result.push_back( point ) ;
		last = point ;
	}
	result.push_back( first );
	return result ;
}


OGRGeometry* CreateAdvancedBuildings::createBuilding(OGRPolygon *ogr_poly, OGRPolygon *ogr_boundary)
{
	char* geo;
	ogr_poly->exportToWkt(&geo);
	std::auto_ptr< SFCGAL::Geometry > g( SFCGAL::io::readWkt(geo));
	SFCGAL::Polygon poly =g->as<SFCGAL::Polygon>();

	std::list<Polygon_2> offest_poly = this->offsetPolygon(poly.toPolygon_2(), 1);
	ogr_boundary->exportToWkt(&geo);
	std::auto_ptr< SFCGAL::Geometry > bg( SFCGAL::io::readWkt(geo));
	SFCGAL::Polygon poly_bg =bg->as<SFCGAL::Polygon>();

	std::list<Polygon_2> offset_bg = this->offsetPolygon(poly_bg.toPolygon_2(), 4);

	typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;
	Pwh_list_2 inter;
	CGAL::intersection(offest_poly.front(), offset_bg.front(), std::back_inserter(inter));

	int n_points = ogr_boundary->getExteriorRing()->getNumPoints();

	std::vector<OGRPoint*> points;
	std::vector<OGRLineString*> boundary_lines;
	for (int i = 0; i < n_points; i++) {
		OGRPoint * p = new OGRPoint();
		ogr_boundary->getExteriorRing()->getPoint(i, p);
		points.push_back(p);

	}
	for (int i = 1; i < n_points; i++) {
		OGRLineString * l = new OGRLineString();
		l->addPoint(points[i-1]);
		l->addPoint(points[i]);
		boundary_lines.push_back(l);
	}

	n_points = ogr_poly->getExteriorRing()->getNumPoints();
	//DM::Logger(DM::Error) << n_points;
	/*foreach(OGRLineString * l, boundary_lines)
		DM::Logger(DM::Error) << l->get_Length();*/

	std::vector<OGRPoint*> parcel_points;

	for (int i = 0; i < n_points; i++) {
		OGRPoint * p = new OGRPoint();
		ogr_poly->getExteriorRing()->getPoint(i, p);
		parcel_points.push_back(p);
	}
	const double PI = 3.141592653589793;
	CGAL::Aff_transformation_2<SFCGAL::Kernel> rotate(CGAL::ROTATION,sin(PI/2),cos(PI/2));

	std::vector<Segment_2> segments;
	for (int i = 1; i < n_points; i++) {
		foreach(OGRLineString * bl, boundary_lines) {
			double d1 =  bl->Distance(parcel_points[i-1]);
			double d2 =  bl->Distance(parcel_points[i]);

			if (d1 < 0.00001 && d2 < 0.000001) {
				OGRLineString  l;
				l.addPoint(parcel_points[i-1]);
				l.addPoint(parcel_points[i]);
				OGRFeature * f = street_access.createFeature();
				f->SetGeometry(&l);
				Point_2 p1(parcel_points[i-1]->getX(), parcel_points[i-1]->getY());
				Point_2 p2(parcel_points[i]->getX(), parcel_points[i]->getY());
				Segment_2 s (p1, p2);
				Vector_2 v = s.to_vector();
				Vector_2 v1 = (p2-p1);
				Vector_2 e1 = v1 / sqrt(CGAL::to_double(v1.squared_length()));
				e1 = rotate(e1);
				Point_2 tmp_p1 = p1 + e1 ;
				Point_2 tmp_p2 = p2 + e1 ;
				DM::Logger(DM::Error) << CGAL::to_double(tmp_p1.x()) << " " << CGAL::to_double(tmp_p1.y()) << " " << CGAL::to_double(tmp_p2.x()) << " " << CGAL::to_double(tmp_p2.y());
				OGRLineString  l1;
				OGRPoint *p_1 =  new OGRPoint(CGAL::to_double(tmp_p1.x()), CGAL::to_double(tmp_p1.y()));
				OGRPoint *p_2 =  new OGRPoint(CGAL::to_double(tmp_p2.x()), CGAL::to_double(tmp_p2.y()));
				l1.addPoint(p_1);
				l1.addPoint(p_2);
				OGRFeature * f1 = street_access.createFeature();
				f1->SetGeometry(&l1);

				//Vector_2 v1 = (p2-p1);
				//Vector_2 e1 = v1 / sqrt(CGAL::to_double(v1.squared_length()));


			}
		}
	}



	/*DM::Logger(DM::Error) << "--";
	foreach(OGRLineString * bl, boundary_lines) {
		foreach(OGRLineString * p, parcel_lines) {
			double d =  bl->Distance(p);
			if (d < 0.000001) {
				OGRFeature * f = street_access.createFeature();
				f->SetGeometry(p);
				DM::Logger(DM::Error) << "street";
			}

		}
	}*/

	Polygon_with_holes_2 p = inter.front();//poly.toPolygon_with_holes_2(true);
	SFCGAL::Polygon poly_build(inter.front());
	Polygon_2 p_c;
	CGAL::convex_hull_2(p.outer_boundary().vertices_begin(), p.outer_boundary().vertices_end(), std::back_inserter(p_c));

	//Cacluate Minimal Rect
	Polygon_2 p_m;
	CGAL::maximum_area_inscribed_k_gon_2(p_c.vertices_begin(), p_c.vertices_end(), 4,std::back_inserter(p_m));
	Point_2 p1 = p_m.vertex(0);
	Point_2 p2 = p_m.vertex(1);

	Point_2 p3 = p_m.vertex(2);

	Vector_2 v1 = (p2-p1);
	Vector_2 v2 = (p3-p2);

	bool v1_bigger = true;
	if (v1.squared_length() < v2.squared_length()) {
		v1_bigger = false;
	}


	std::string wkt = poly_build.asText(9).c_str();

	char * writable_wr = new char[wkt.size() + 1]; //Note not sure if memory hole?
	std::copy(wkt.begin(), wkt.end(), writable_wr);
	writable_wr[wkt.size()] = '\0';

	OGRGeometry * ogr_poly_build;

	OGRErr err = OGRGeometryFactory::createFromWkt(&writable_wr, 0, &ogr_poly_build);

	//Identify street sides

	Vector_2 e1 = v1 / sqrt(CGAL::to_double(v1.squared_length()));
	Vector_2 e2 = v2 / sqrt(CGAL::to_double(v2.squared_length()));

	OGRPoint ct;
	ogr_poly_build->Centroid(&ct);

	Point_2 centre(ct.getX(), ct.getY());

	double w = (!v1_bigger) ? this->width/2. : this->height/2.;
	double h = (!v1_bigger) ? this->height/2. : this->width/2.;

	Polygon_with_holes_2 footprint;
	footprint.outer_boundary().push_back( centre - e1*w - e2*h );
	footprint.outer_boundary().push_back( centre + e1*w - e2*h );
	footprint.outer_boundary().push_back( centre + e1*w + e2*h );
	footprint.outer_boundary().push_back( centre - e1*w + e2*h );

	SFCGAL::Polygon f(footprint);

	return addToSystem(f);
}

OGRGeometry *  CreateAdvancedBuildings::addToSystem(SFCGAL::Polygon & poly)
{

	std::string wkt = poly.asText(9).c_str();

	char * writable_wr = new char[wkt.size() + 1]; //Note not sure if memeory hole?
	std::copy(wkt.begin(), wkt.end(), writable_wr);
	writable_wr[wkt.size()] = '\0';

	OGRGeometry * ogr_poly;

	OGRErr err = OGRGeometryFactory::createFromWkt(&writable_wr, 0, &ogr_poly);

	if (!ogr_poly->IsValid()) {
		DM::Logger(DM::Warning) << "Geometry is not valid!";
		return 0;
	}
	if (ogr_poly->IsEmpty()) {
		DM::Logger(DM::Warning) << "Geometry is empty ";
		DM::Logger(DM::Warning) << "OGR Error " << err;
		DM::Logger(DM::Warning) << poly.asText(9);
		return 0;
	}

	return ogr_poly;
}

CreateAdvancedBuildings::CreateAdvancedBuildings()
{
	GDALModule = true;

	this->width = 10;
	this->addParameter("width", DM::DOUBLE, &this->width);

	this->height = 10;
	this->addParameter("length", DM::DOUBLE, &this->height);

	this->residential_units = 1;
	this->addParameter("residential_units", DM::INT, &this->residential_units);

	parcel = DM::ViewContainer("parcel", DM::FACE, DM::READ);
	parcel.addAttribute("cityblock_id", DM::Attribute::INT, DM::READ);

	building = DM::ViewContainer("building", DM::FACE, DM::WRITE);
	building.addAttribute("residential_units", DM::Attribute::INT, DM::WRITE);

	cityblock = DM::ViewContainer("cityblock", DM::FACE, DM::READ);

	street_access = DM::ViewContainer("street_access", DM::EDGE, DM::READ);

	std::vector<DM::ViewContainer*> data_stream;
	data_stream.push_back(&parcel);
	data_stream.push_back(&building);
	data_stream.push_back(&cityblock);
	data_stream.push_back(&street_access);

	this->registerViewContainers(data_stream);
}

void CreateAdvancedBuildings::run()
{
	parcel.resetReading();

	OGRFeature * f;

	while (f = parcel.getNextFeature()) {
		OGRPolygon * geo = (OGRPolygon *)f->GetGeometryRef();
		if (!geo)
			continue;

		//Offset outer polygon with building offset (off to street)
		OGRFeature * cb = cityblock.getFeature(f->GetFieldAsInteger("cityblock_id"));
		OGRPolygon * cb_geom = (OGRPolygon *) cb->GetGeometryRef();

		//Identify outside of Polygon

		OGRGeometry * building_geo  = this->createBuilding(geo, cb_geom);

		if (!building_geo)
			continue;
		//Create Feature
		OGRFeature * b = building.createFeature();
		b->SetGeometry(building_geo);
		b->SetField("residential_units", this->residential_units);
		OGRGeometryFactory::destroyGeometry(building_geo);
	}
}
