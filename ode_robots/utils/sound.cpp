/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2007-11-07 13:28:52  martius
 *   sound representation
 *
 *   Revision 1.2  2006/07/14 12:23:56  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/06/22 11:38:07  robot3
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include "sound.h"
#include "osghandle.h"
#include "osgprimitive.h"

namespace lpzrobots {
  
  Sound::~Sound(){ 
    if(visual) delete visual; 
  }


  void Sound::render(const OsgHandle& osgHandle){
    if(!visual){
      visual = new OSGSphere((intensity+1.0)/4.0);
      visual->init(osgHandle.changeColor(Color(255-int((frequency+1.0)*128.0),
					       0,int((frequency+1.0)*128.0),0.4)));
      visual->setMatrix(osg::Matrix::translate(pos.x(), pos.y(), pos.z()+1));
    }
  }

}