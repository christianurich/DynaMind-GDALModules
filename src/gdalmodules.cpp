/**
 * @file
 * @author  Chrisitan Urich <christian.urich@gmail.com>
 * @version 1.0
 * @section LICENSE
 *
 * This file is part of DynaMind
 *
 * Copyright (C) 2014  Christian Urich

 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */
#include "dmnodefactory.h"
#include "dmmoduleregistry.h"
#include "gdalimportdata.h"
#include "parceling.h"
#include "gdalcreaterectangle.h"
#include "lsystems.h"
#include "nonspatiallinking.h"
#include "gdalattributecalculator.h"
#include "gdalspatiallinking.h"
#include "gdalerase.h"
#include "gdaloffset.h"
#include "gdalparcelsplit.h"
#include "gdalpublishresults.h"
#include "gdalgeometricattributes.h"
#include "gdalrandomselector.h"
#include "gdalremovecomponets.h"
#include "gdalcopyelementtoview.h"
#include "gdalcreatebuilding.h"
#include "gdalcreatecentroids.h"
#include "gdalcreatehouseholds.h"
#include "gdaldmswmm.h"
#include "gdaljoinnearestneighbour.h"
#include "gdallogattriubtes.h"
#include "gdalcreateneighbourhoodtable.h"
#include "gdalclusterneighbourhood.h"
#include "gdalmergefaces.h"
#include "gdalhotstarter.h"
#include "gdalclusternetwork.h"
#include "directnetwork.h"
#include "gdaljoincluster.h"
#include "gdalextractnodes.h"
#include "createadvancedbuildings.h"

using namespace std;
extern "C" void DM_HELPER_DLL_EXPORT  registerModules(DM::ModuleRegistry *registry) {
	registry->addNodeFactory(new DM::NodeFactory<GDALImportData>());
	registry->addNodeFactory(new DM::NodeFactory<GDALParceling>());
	registry->addNodeFactory(new DM::NodeFactory<GDALCreateRectangle>());
	registry->addNodeFactory(new DM::NodeFactory<LSystems>());
	registry->addNodeFactory(new DM::NodeFactory<NonSpatialLinking>());
	registry->addNodeFactory(new DM::NodeFactory<GDALSpatialLinking>());
	registry->addNodeFactory(new DM::NodeFactory<GDALAttributeCalculator>());
	registry->addNodeFactory(new DM::NodeFactory<GDALErase>());
	registry->addNodeFactory(new DM::NodeFactory<GDALOffset>());
	registry->addNodeFactory(new DM::NodeFactory<GDALParcelSplit>());
	registry->addNodeFactory(new DM::NodeFactory<GDALPublishResults>());
	registry->addNodeFactory(new DM::NodeFactory<GDALGeometricAttributes>());
	registry->addNodeFactory(new DM::NodeFactory<GDALRandomSelector>());
	registry->addNodeFactory(new DM::NodeFactory<GDALRemoveComponets>());
	registry->addNodeFactory(new DM::NodeFactory<GDALCopyElementToView>());
	registry->addNodeFactory(new DM::NodeFactory<GDALCreateBuilding>());
	registry->addNodeFactory(new DM::NodeFactory<GDALCreateCentroids>());
	registry->addNodeFactory(new DM::NodeFactory<GDALCreateHouseholds>());
	registry->addNodeFactory(new DM::NodeFactory<GDALDMSWMM>());
	registry->addNodeFactory(new DM::NodeFactory<GDALJoinNearestNeighbour>());
	registry->addNodeFactory(new DM::NodeFactory<GDALLogAttriubtes>());
	registry->addNodeFactory(new DM::NodeFactory<GDALCreateNeighbourhoodTable>());
	registry->addNodeFactory(new DM::NodeFactory<GDALClusterNeighbourhood>());
	registry->addNodeFactory(new DM::NodeFactory<GDALMergeFaces>());
	registry->addNodeFactory(new DM::NodeFactory<GDALHotStarter>());
	registry->addNodeFactory(new DM::NodeFactory<GDALClusterNetwork>());
	registry->addNodeFactory(new DM::NodeFactory<GDALDirectNetwork>());
	registry->addNodeFactory(new DM::NodeFactory<GDALJoinCluster>());
	registry->addNodeFactory(new DM::NodeFactory<GDALExtractNodes>());
	registry->addNodeFactory(new DM::NodeFactory<CreateAdvancedBuildings>());
}
