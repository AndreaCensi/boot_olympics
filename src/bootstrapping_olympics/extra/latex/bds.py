from . import load_report_phase
from StringIO import StringIO
from latex_gen import latex_fragment
from reprep import Node
from reprep.constants import MIME_PNG
import scipy.io
import sys
from . import create_robot_figure


def bds_learn_reportA(id_set, agent, robot, draft=False):

    report = load_report_phase(id_set=id_set,
                         agent=agent, robot=robot, phase='learn')

    report_predict = load_report_phase(id_set, agent, robot, 'predict')
    y_dot_corr = report_predict['y_dot/figure1/correlation']
#    sys.stderr.write(report.format_tree())

    T = report['estimator/tensors/T/value'].raw_data
    ncmd = T.shape[2]
    
    assert isinstance(report, Node)
    
    def write_graphics_data(el, data_node, gid, width=None):
        # Add here low resolution
        # In draft mode, do not output pngs
        use_draft = draft and MIME_PNG == data_node.mime
        
        if not use_draft:
            el.graphics_data(data_node.raw_data, data_node.mime,
                                  width=width, gid=gid)
        else:
            el.rule(tsize, tsize, color='gray')
    
    def boxed(el, data_node, gid, width=None):
        with el.tightbox() as box:
            write_graphics_data(box, data_node, gid, width=width)

    graphics_path = 'snippets/figures'
    with latex_fragment(sys.stdout, graphics_path=graphics_path) as frag:
        caption = 'Report for robot %r, agent %r. ' % (robot, agent)
        label = 'fig:%s-%s-%s-learn-rA' % (id_set, robot, agent)
        
        with frag.figure(caption=caption, label=label, placement="p") as fig:
            tsize = '3cm'

#            fig.hfill()
#            with fig.subfigure(caption="$\Tcove^{-1}_{sv}$",
#                               label='%s-%s' % (label, 'Pinv'))  as sub:
#
#                boxed(sub, report['estimator/tensors/P_inv_cond/value/posneg'],
#                      '%s-%s-%s-P_inv_cond' % (id_set, robot, agent),
#                      width=tsize)
            
            with fig.subfigure(caption="\\texttt{%s}" % robot,
                               label='%s-%s' % (label, 'vehicle')) as sub:
        
                with sub.minipage("3cm", align='b') as mp:
                    mp.tex('\\vspace{0pt}\n')
                    mp.tex('\\centering')
                    create_robot_figure(mp, id_set, robot)

            fig.hfill()
            for i in range(ncmd):
                subcaption = '$\TUe^{s\,%d}$' % i
                sublabel = '%s-U%d' % (label, i)
                Ui_im = report['estimator/tensors/U/slices/%d/figure1/plotBna' % i]
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    gid = '%s-%s-%s-U%d' % (id_set, robot, agent, i)
                    write_graphics_data(sub, Ui_im, gid, width=tsize)
                fig.hfill()

            fig.parbreak()
            
            fig.hfill()
            
            with fig.subfigure(caption="$\Tcove^{sv}$",
                               label='%s-%s' % (label, 'P'))  as sub:
                boxed(sub, report['estimator/tensors/P/value/posneg'],
                       '%s-%s-%s-P' % (id_set, robot, agent),
                       width=tsize)

            fig.hfill()

            for i in range(ncmd):
                subcaption = '$\TTe^{s\,v\,%d}$' % i
                sublabel = '%s-T%d' % (label, i)
                
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    boxed(sub, report['estimator/tensors/T/slices/%d/value/posneg' % i],
                          '%s-%s-%s-T%d' % (id_set, robot, agent, i),
                          width=tsize)

                fig.hfill()
                
            fig.hfill()
              
            fig.parbreak()
            
            fig.hfill()
            fig.rule(tsize, '0pt')
            
            fig.hfill()
            for i in range(ncmd):
                subcaption = '$\TNe^{s\,%d}$' % i
                sublabel = '%s-N%d' % (label, i)
                Ni_im = report['estimator/model/N/slices/%d/figure1/plotBna' % i]
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    gid = '%s-%s-%s-N%d' % (id_set, robot, agent, i)
                    write_graphics_data(sub, Ni_im, gid, width=tsize)

                fig.hfill()
                
            fig.parbreak()
            
            fig.hfill()
            with fig.subfigure(caption="correlation",
                               label='%s-%s' % (label, 'corr'))  as sub:
                write_graphics_data(sub, y_dot_corr,
                             '%s-%s-%s-y_dot_corr' % (id_set, robot, agent),
                                     width=tsize)
            
            fig.hfill()
            for i in range(ncmd):
                subcaption = '$\TMe^s_{v\,%d}$' % i
                sublabel = '%s-M%d' % (label, i)
                with fig.subfigure(caption=subcaption, label=sublabel) as sub:
                    boxed(sub, report['estimator/model/M/slices/%d/value/posneg' % i],
                          '%s-%s-%s-M%d' % (id_set, robot, agent, i),
                          width=tsize)

                fig.hfill()

            if False:
                data = {}
                data['T'] = report['estimator/tensors/T/value'].raw_data
                data['U'] = report['estimator/tensors/U/value'].raw_data
                data['M'] = report['estimator/model/M/value'].raw_data
                data['N'] = report['estimator/model/N/value'].raw_data
                data['P'] = report['estimator/tensors/P/value'].raw_data
                data['P_inv_cond'] = report['estimator/tensors/P_inv_cond/value/posneg'].raw_data
                mat = StringIO()
                scipy.io.savemat(mat, data, oned_as='row')
                matdata = mat.getvalue()
                
                fig.parbreak()
                fig.textattachfile('%s-%s-%s-learn.mat' % (id_set, robot, agent),
                                   matdata, 'Matlab format')
       
