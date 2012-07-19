from . import (create_robot_figure, load_report_phase, png_data, jpg_data,
    get_sensel_pgftable, get_predict_corr, get_resources_dir)
from contracts import contract
from latex_gen import latex_fragment
from reprep import posneg, rgb_zoom, MIME_PNG, MIME_JPG, Node, MIME_PLAIN
import numpy as np
import sys

    
@contract(V='array[NxN]')
def display_tensor_with_zoom(fig, V, gid, label, width, xlabel, ylabel, zoom=16,
                             x=0.15, w=0.03):
    n = V.shape[0]
    rgb = posneg(V)
    
    if n > 50:
        fig.save_graphics_data(jpg_data(rgb), MIME_JPG, gid)
    else:
        rgb = rgb_zoom(rgb, 16)
        fig.save_graphics_data(png_data(rgb), MIME_PNG, gid)
    
    if n > 100:
        xp = int(x * n)
        wp = int(w * n)
        cut_rgb = rgb_zoom(rgb[xp:(xp + wp), xp:(xp + wp), :], zoom)
        fig.save_graphics_data(png_data(cut_rgb), MIME_PNG, gid + '-zoom')
        fig.tex('\\tensorFigure{%s}{%s}{%s}{%s}{%s}{%s}{%s}' % 
                      (gid, width, xlabel, ylabel, label, x, w))
    else:
        fig.tex('\\tensorFigure{%s}{%s}{%s}{%s}{%s}{%s}{%s}' % 
                (gid, width, xlabel, ylabel, label, '', ''))


@contract(R='array[N]')
def display_correlation(fig, R, gid, width):
    table = get_sensel_pgftable(R, 'corr', 'Correlation for %s' % gid)
    fig.save_graphics_data(table, MIME_PLAIN, gid)
    fig.tex('\\corrFigure{%s}{%s}' % (gid, width))
    

@contract(V='array[NxNxK]')
def display_3tensor(fig, V, gid_pattern, label_patterns, caption_patterns,
                   width, label, xlabel, ylabel):
    fig.hfill()
    for i in range(V.shape[2]):
        with fig.subfigure(caption='$%s$' % (caption_patterns % i),
                           label=(label_patterns % i)) as sub:
            display_tensor_with_zoom(sub, V[:, :, i], width=width,
                                     gid=gid_pattern % i,
                                     label=label % i, xlabel=xlabel,
                                     ylabel=ylabel)
            fig.hfill()


@contract(V='array[NxK]')
def display_k_tensor(fig, V, gid_pattern, label_patterns, caption_patterns,
                     width, xlabel, ylabel):
    fig.hfill()
    for i in range(V.shape[1]):
        with fig.subfigure(caption=(caption_patterns % i),
                           label=(label_patterns % i)) as sub:
            # Nonnormalized
            gid = gid_pattern % i
            table = get_sensel_pgftable(V[:, i], 'value', gid) 
            sub.save_graphics_data(table, MIME_PLAIN, gid)
            # Normalized
            gidn = gid_pattern % i + '-norm'
            Vni = V[:, i] / np.max(np.abs(V))
            table = get_sensel_pgftable(Vni, 'valuen', gidn) 
            sub.save_graphics_data(table, MIME_PLAIN, gidn)
        
            sub.tex('\\tensorOnePlot{%s}{%s}{%s}{%s}{%s}' % 
                  (gid, width, xlabel, ylabel, ''))

            sub.hfill()
      
      
def fig_predict_corr(fig, id_set, agent, robot, width):
    prefix = '%s-%s-%s' % (id_set, robot, agent)
    gid = prefix + '-pred-corr'
    R = get_predict_corr(id_set, agent, robot)
    display_correlation(fig, R, gid, width)
    
    
def bds_learn_reportA(id_set, agent, robot, width='3cm'):
    prefix = '%s-%s-%s' % (id_set, robot, agent)
    report = load_report_phase(id_set=id_set,
                         agent=agent, robot=robot, phase='learn')
    
    assert isinstance(report, Node)

    def tensor_figure(where, gid, xlabel, ylabel, width, label):
        where.tex('\\tensorFigure{%s}{%s}{%s}{%s}{%s}{}{}' % 
                  (gid, width, xlabel, ylabel, label))
        
    def save_data(node, gid):
        fig.save_graphics_data(node.raw_data, node.mime, gid)

    with latex_fragment(sys.stdout, graphics_path=get_resources_dir()) as frag:
        caption = 'Report for robot %r, agent %r. ' % (robot, agent)
        label = 'fig:%s-%s-%s-learn-rA' % (id_set, robot, agent)
        
        with frag.figure(caption=caption, label=label, placement="p") as fig:
            tsize = '3cm'

            with fig.subfigure(caption="\\texttt{%s}" % robot,
                               label='%s-%s' % (label, 'vehicle')) as sub:
        
                with sub.minipage("3cm", align='b') as mp:
                    mp.tex('\\vspace{0pt}\n')
                    mp.tex('\\centering')
                    create_robot_figure(mp, id_set, robot)

            display_k_tensor(fig, report['estimator/tensors/U/value'].raw_data,
                             gid_pattern='%s-U%%d' % (prefix),
                             label_patterns='fig:%s' % prefix + '-U%d',
                             caption_patterns='$\TUe^{s}_{%d}$',
                             width=tsize, xlabel='s', ylabel='\TUe^{s}')

            fig.parbreak()
            
            fig.hfill()

            with fig.subfigure(caption="$\Tcove^{sv}$",
                               label='%s-%s' % (label, 'P'))  as sub:
                gid = '%s-P' % (prefix)
                save_data(report['estimator/tensors/P/png'], gid)
                tensor_figure(sub, gid=gid, xlabel='s', ylabel='v', width=width,
                              label='P^{sv}')
                
            display_3tensor(fig, V=report['estimator/tensors/T/value'].raw_data,
                            gid_pattern=prefix + '-T%d',
                            label_patterns='fig:%s' % prefix + '-T%d',
                            caption_patterns='\TTe^{s\,v\,%d}',
                            width=width,
                            label='\TTe^{s\,v\,%d}',
                            xlabel='s', ylabel='v')
                                 
            fig.parbreak()
            
            fig.hfill()
            fig.rule(tsize, '0pt')
            
            display_k_tensor(fig, report['estimator/model/N/value'].raw_data,
                             gid_pattern=prefix + '-N%d',
                             label_patterns='fig:%s' % prefix + '-N%d',
                             caption_patterns='$\TNe^{s}_{%d}$',
                             width=tsize, xlabel='s', ylabel='\TNe^{s}')

            fig.parbreak()
            
            fig.hfill()
            with fig.subfigure(caption="correlation",
                               label='%s-%s' % (label, 'corr'))  as sub:
                fig_predict_corr(sub, id_set, agent, robot, width)
            
            display_3tensor(fig, V=report['estimator/model/M/value'].raw_data,
                            gid_pattern=prefix + '-M%d',
                            label_patterns='fig:%s' % prefix + '-M%d',
                            caption_patterns='\TMe^s_{v\,%d}',
                            width=width,
                            label='\TMe^s_{v\,%d}',
                            xlabel='s', ylabel='v')


def get_bds_summary(id_set, agent, robot):
    report = load_report_phase(id_set=id_set,
                         agent=agent, robot=robot, phase='learn')
    data = {}
    data['T'] = report['estimator/tensors/T/value'].raw_data
    data['U'] = report['estimator/tensors/U/value'].raw_data
    data['M'] = report['estimator/model/M/value'].raw_data
    data['N'] = report['estimator/model/N/value'].raw_data
    data['P'] = report['estimator/tensors/P/value'].raw_data
    data['P_inv_cond'] = report['estimator/tensors/P_inv_cond/value/posneg'].raw_data
#                mat = StringIO()
#                scipy.io.savemat(mat, data, oned_as='row')
#                matdata = mat.getvalue()
#                
#                fig.parbreak()
#                fig.textattachfile('%s-%s-%s-learn.mat' % (id_set, robot, agent),
#                                   matdata, 'Matlab format')
#       
